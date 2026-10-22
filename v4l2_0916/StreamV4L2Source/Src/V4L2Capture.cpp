#include "V4L2Capture.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include "FunctionProfiler.h"

namespace V4L2StreamLib
{
    // 定义最大平面数
#ifndef VIDEO_MAX_PLANES
#define VIDEO_MAX_PLANES 8
#endif


    V4L2Capture::V4L2Capture(const std::string& device_path)
        : device_path_(device_path)
    {

    }

    V4L2Capture::~V4L2Capture()
    {

    }

    bool V4L2Capture::Initialize(const VideoParams& param)
    {
        encodeParams_ = param;
        if (!OpenDevice()) {
            std::cerr << "Failed to open V4L2 device: " << device_path_ << std::endl;
            return false;
        }

        if (!PrintCapabilities()) {
            std::cerr << "Failed to print device capabilities" << std::endl;
            CloseDevice();
            return false;
        }

        if (!SetFormat()) {
            std::cerr << "Failed to set V4L2 format" << std::endl;
            CloseDevice();
            return false;
        }

        if (!InitV4L2Buffers()) {
            std::cerr << "Failed to initialize V4L2 buffers" << std::endl;
            CloseDevice();
            return false;
        }

        if (!StartStreaming()) {
            std::cerr << "Failed to start V4L2 streaming" << std::endl;
            CloseDevice();
            return false;
        }

        return true;
    }

    void V4L2Capture::UnInit()
    {
        StopStreaming();
        CloseDevice();
    }

    void V4L2Capture::SetStreamCallback(StreamFrameCallback callback, void* userData)
    {
        videoEncoder_.SetStreamCallback(callback, userData);
    }

    void V4L2Capture::SetStreamCallback(V4L2FrameCallback callback, void* userData)
    {
        frameCallback_ = callback;
        frameCallbackParam_ = userData;
    }

    bool V4L2Capture::StartCapture()
    {
        if (fd_v4l2_ == -1)
            return false;

        if (is_capturing_.load()) {
            std::cerr << "Already capturing!" << std::endl;
            return true;
        }

        if (!videoEncoder_.InitEncoder(encodeParams_))
        {
            std::cerr << "init encoder error" << std::endl;
            return false;
        }

        is_capturing_ = true;
        capture_thread_ = std::thread(&V4L2Capture::CaptureLoop, this);

        std::cout << "Started capturing" << std::endl;
        return true;
    }

    void V4L2Capture::StopCapture()
    {
        if (!is_capturing_.load()) {
            return;
        }

        is_capturing_ = false;

        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
        videoEncoder_.Close();
    }


    bool V4L2Capture::OpenDevice() {
        fd_v4l2_ = open(device_path_.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd_v4l2_ == -1) {
            perror("Opening video device");
            return false;
        }

        return true;
    }

    bool V4L2Capture::PrintCapabilities() {
        struct v4l2_capability cap;
        if (ioctl(fd_v4l2_, VIDIOC_QUERYCAP, &cap) < 0) {
            perror("Querying Capabilities");
            return false;
        }

        std::cout << "Driver Name: " << cap.driver << std::endl;
        std::cout << "Card Name: " << cap.card << std::endl;
        std::cout << "Bus Info: " << cap.bus_info << std::endl;
        std::cout << "Version: " << ((cap.version >> 16) & 0xFF) << "."
            << ((cap.version >> 8) & 0xFF) << "."
            << (cap.version & 0xFF) << std::endl;
        std::cout << "Capabilities: " << std::hex << cap.capabilities << std::dec << std::endl;
        return true;
    }

    bool V4L2Capture::SetFormat() {
        // 首先尝试查询设备支持的所有格式
        struct v4l2_fmtdesc fmtdesc = {};
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmtdesc.index = 0;

        std::vector<uint32_t> supported_formats;
        while (ioctl(fd_v4l2_, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
            supported_formats.push_back(fmtdesc.pixelformat);
            std::cout << "Supported format [" << fmtdesc.index << "]: ";
            // 打印像素格式
            char fmt_str[5] = { 0 };
            memcpy(fmt_str, &fmtdesc.pixelformat, 4);
            std::cout << fmt_str << " (" << fmtdesc.description << ")" << std::endl;

            fmtdesc.index++;
        }

        //Although RK3588 supports NV12 Multiplane, but it seems only 1 plane is used!!!

        struct v4l2_format fmt = {};
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        fmt.fmt.pix_mp.width = encodeParams_.width;
        fmt.fmt.pix_mp.height = encodeParams_.height;
        fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
        fmt.fmt.pix_mp.pixelformat = input_format_; //force to use this NV12
        fmt.fmt.pix_mp.num_planes = 1;

        if (ioctl(fd_v4l2_, VIDIOC_S_FMT, &fmt) < 0)
        {
            perror("Setting Pixel Format to NV12 failed");
            return false;
        }

        // 更新实际的宽高（驱动可能会调整参数）
        encodeParams_.width = fmt.fmt.pix_mp.width;
        encodeParams_.height = fmt.fmt.pix_mp.height;

        std::cout << "Set format to " << encodeParams_.width << "x" << encodeParams_.height << " with pixel format: ";
        // 打印实际使用的格式
        char fmt_str[5] = { 0 };
        memcpy(fmt_str, &input_format_, 4);
        std::cout << fmt_str << std::endl;
        std::cout << "Actual planes: " << static_cast<int>(fmt.fmt.pix_mp.num_planes) << std::endl;
        std::cout << "Plane 0 size: " << fmt.fmt.pix_mp.plane_fmt[0].sizeimage << std::endl;
        return true;
    }

    bool V4L2Capture::InitV4L2Buffers() {
        struct v4l2_requestbuffers req = {};
        req.count = n_buffers_;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_v4l2_, VIDIOC_REQBUFS, &req) < 0) {
            perror("Requesting buffer");
            return false;
        }
        else
        {
            std::cout << "Requested " << req.count << " buffers successfully" << std::endl;
        }

        buffers_ = new buffer[req.count];

        for (int i = 0; i < req.count; ++i)
        {
            struct v4l2_plane planes[VIDEO_MAX_PLANES] = {}; // 1 plane for RK3588 NV12
            struct v4l2_buffer buf = {};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;
            buf.m.planes = planes;
            buf.length = 1; // 1 plane for NV12

            if (ioctl(fd_v4l2_, VIDIOC_QUERYBUF, &buf) < 0) {
                perror("Query buffer for MPLANE");
                return false;
            }

            std::cout << "Buffer " << i << ":" << std::endl;
            std::cout << "  NV12 Plane Offset: " << buf.m.planes[0].m.mem_offset << std::endl;
            std::cout << "  NV12 Plane Length: " << buf.m.planes[0].length << std::endl;

            // 对于多平面缓冲区，我们需要映射总的内存大小
            // NV12格式通常有两个平面：Y平面和UV平面，但RK3588这里只有一个
            buffers_[i].length = buf.m.planes[0].length;
            buffers_[i].start = mmap(nullptr, buffers_[i].length,
                PROT_READ | PROT_WRITE, MAP_SHARED,
                fd_v4l2_, buf.m.planes[0].m.mem_offset);

            if (buffers_[i].start == MAP_FAILED) {
                perror("Mapping buffer for MPLANE");
                return false;
            }

            // 将所有缓冲区加入队列
            if (ioctl(fd_v4l2_, VIDIOC_QBUF, &buf) < 0) {
                perror("Queuing buffer for MPLANE");
                return false;
            }
        }
        return true;
    }

    bool V4L2Capture::StartStreaming() {
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        if (ioctl(fd_v4l2_, VIDIOC_STREAMON, &type) < 0) {
            perror("Start streaming");
            return false;
        }
        else
        {
            std::cout << "Started streaming successfully" << std::endl;
        }
        return true;
    }

    bool V4L2Capture::StopStreaming() {
        enum v4l2_buf_type type;
        type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

        if (ioctl(fd_v4l2_, VIDIOC_STREAMOFF, &type) < 0) {
            perror("Stop streaming");
            return false;
        }
        return true;
    }

    void V4L2Capture::CloseDevice()
    {
        if (buffers_) {
            for (int i = 0; i < n_buffers_; ++i) {
                munmap(buffers_[i].start, buffers_[i].length);
            }
            delete[] buffers_;
            buffers_ = nullptr;
        }

        if (fd_v4l2_ != -1) {
            close(fd_v4l2_);
            fd_v4l2_ = -1;
        }
    }


    void V4L2Capture::CaptureLoop()
    {
        auto tpNow = std::chrono::system_clock::now();
        std::time_t tNow = std::chrono::system_clock::to_time_t(tpNow);
        firstPts_ = tNow * 1000;

        int frame_count = 0;
        auto start_time = std::chrono::high_resolution_clock::now();

        while (is_capturing_.load()) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd_v4l2_, &fds);

            struct timeval tv;
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            int r = select(fd_v4l2_ + 1, &fds, nullptr, nullptr, &tv);
            if (r == -1) {
                perror("Select failed");
                break;
            }
            else if (r == 0) {
                std::cerr << "Select timeout" << std::endl;
                continue;
            }

            // 根据格式类型确定buffer类型
            struct v4l2_buffer buf = {};
            struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
            // MPLANE类型
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.m.planes = planes;
            buf.length = 1;

            if (ioctl(fd_v4l2_, VIDIOC_DQBUF, &buf) < 0) {
                perror("Retrieving frame");
                break;
            }

            // 编码帧
            AVFrame* frame = av_frame_alloc();
            if (!frame) {
                std::cerr << "Could not allocate frame" << std::endl;
                break;
            }

            frame->width = encodeParams_.width;
            frame->height = encodeParams_.height;
            frame->format = AV_PIX_FMT_NV12;

            int ret = av_frame_get_buffer(frame, 32);
            if (ret < 0) {
                std::cerr << "Could not allocate frame data" << std::endl;
                av_frame_free(&frame);
                break;
            }

            // 直接使用 V4L2 缓冲区数据，避免拷贝
            frame->data[0] = static_cast<uint8_t*>(buffers_[buf.index].start);                    // Y 平面
            frame->data[1] = static_cast<uint8_t*>(buffers_[buf.index].start) + encodeParams_.width * encodeParams_.height; // UV 平面
            frame->linesize[0] = encodeParams_.width;
            frame->linesize[1] = encodeParams_.width;

            // 设置时间戳
            // 根据音视频编码时间戳与时间基处理规范，使用帧计数作为时间戳
            // 避免使用微秒级时间戳导致值过大的问题
            //frame->pts = frame_count;
            struct timeval v4l2_timestamp = buf.timestamp;
            uint64_t v4l2_capture_us = (uint64_t)v4l2_timestamp.tv_sec * 1000000LL + (uint64_t)v4l2_timestamp.tv_usec;
            //std::cout << "V4L2 timestamp: usec=" << v4l2_capture_us << std::endl;
            int64_t v4l2_pts_us = (int64_t)v4l2_timestamp.tv_sec * 1000000LL + (int64_t)v4l2_timestamp.tv_usec;
			frame->pts = (v4l2_pts_us - firstV4L2_pts_us_)/1000 + firstPts_;

            if (encodeParams_.is_compressed == 1)
            {
                videoEncoder_.OnFrame(this, frame);
            }
            else
            {
                PROFILE_FUNCTION_START(encode_frame);
                if (frameCallback_)
                    frameCallback_(0, frame, v4l2_capture_us, frameCallbackParam_);
                PROFILE_FUNCTION_END(encode_frame);
            }

            av_frame_free(&frame);

            // 将缓冲区重新排队
            if (buf.type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
                buf.m.planes = planes;
                buf.length = 1;
            }

            if (ioctl(fd_v4l2_, VIDIOC_QBUF, &buf) < 0) {
                perror("Requeuing buffer");
                break;
            }

            frame_count++;
            if (frame_count % 100 == 0) {
                //Calculate and print last 100 frames FPS
                auto current_time = std::chrono::high_resolution_clock::now();
                double elapsed_seconds = std::chrono::duration_cast<std::chrono::duration<double>>(current_time - start_time).count();
                double fps = 100 / elapsed_seconds;
                std::cout << "Encoded " << frame_count << " frames, latest 100 frames FPS: " << fps << std::endl;
                start_time = current_time;
            }
        }

    }
    // 新增方法：获取帧数据
    bool V4L2Capture::GetFrameData(AVFrame*& frame) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_v4l2_, &fds);

        struct timeval tv;
        tv.tv_sec = 1;
        tv.tv_usec = 0;

        int r = select(fd_v4l2_ + 1, &fds, nullptr, nullptr, &tv);
        if (r == -1) {
            perror("Select failed in GetFrameData");
            return false;
        }
        else if (r == 0) {
            //std::cerr << "Select timeout in GetFrameData" << std::endl;
            return false; // 超时无数据
        }

        // 获取V4L2缓冲区
        struct v4l2_buffer buf = {};
        struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(fd_v4l2_, VIDIOC_DQBUF, &buf) < 0) {
            perror("Retrieving frame in GetFrameData");
            return false;
        }

        // 创建AVFrame并引用V4L2数据
        frame = av_frame_alloc();
        if (!frame) {
            std::cerr << "Could not allocate frame in GetFrameData" << std::endl;
            // 将缓冲区重新排队
            if (ioctl(fd_v4l2_, VIDIOC_QBUF, &buf) < 0) {
                perror("Requeuing buffer after alloc fail");
            }
            return false;
        }

        frame->width = encodeParams_.width;
        frame->height = encodeParams_.height;
        frame->format = AV_PIX_FMT_NV12;
        frame->data[0] = static_cast<uint8_t*>(buffers_[buf.index].start);                    // Y 平面
        frame->data[1] = static_cast<uint8_t*>(buffers_[buf.index].start) + encodeParams_.width * encodeParams_.height; // UV 平面
        frame->linesize[0] = encodeParams_.width;
        frame->linesize[1] = encodeParams_.width;

        // 存储缓冲区索引以便后续释放
        frame->opaque = reinterpret_cast<void*>(static_cast<intptr_t>(buf.index));

        current_buffer_idx_.store(buf.index);
        return true;
    }

    void V4L2Capture::ReleaseFrame(int buffer_idx) {
        struct v4l2_buffer buf = {};
        struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = buffer_idx;
        buf.m.planes = planes;
        buf.length = 1;

        if (ioctl(fd_v4l2_, VIDIOC_QBUF, &buf) < 0) {
            perror("Requeuing buffer in ReleaseFrame");
        }
    }

}