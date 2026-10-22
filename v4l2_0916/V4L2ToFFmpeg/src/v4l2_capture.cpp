#include "v4l2_capture.h"
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <errno.h>
#include <cstring>
#include <chrono>
#include <mutex>
#include <sys/stat.h>
#include <sstream>
#include <cstdio>
#include <vector>
#include <algorithm>

// 使用项目中已有的rockchip头文件
extern "C" {
#include <rockchip/rk_mpi.h>
#include <rockchip/rk_type.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_packet.h>
}

V4L2Capture::V4L2Capture(const std::string& timestamp_device_path,
                                                 const std::string& capture_device_path)
        : timestamp_device_path_(timestamp_device_path),
            capture_device_path_(capture_device_path),
      fd_v4l2_(-1),
      buffers_(nullptr),
      n_buffers_(4),
            fd_timestamp_(-1),
            timestamp_buffers_(nullptr),
            n_timestamp_buffers_(4),
      width_(1280),
      height_(720),
      fps_(30),
      input_format_(V4L2_PIX_FMT_NV12),  // 默认使用NV12格式
      // 初始化MPP相关成员变量
      mpp_enc_(nullptr),
      mpp_api_(nullptr),
      mpp_enc_cfg_(nullptr),
      mpp_frm_(nullptr),
      mpp_pkt_(nullptr),
      is_capturing_(false),
      firstPts_(0) {
}

V4L2Capture::~V4L2Capture() {
    StopCapture();
    if (fd_v4l2_ != -1) {
        StopStreaming(fd_v4l2_);
    }
    if (fd_timestamp_ != -1) {
        StopStreaming(fd_timestamp_);
    }
    CloseDevice();
}

bool V4L2Capture::Initialize(int width, int height, int fps) {
    width_ = width;
    height_ = height;
    fps_ = fps;

    if (!OpenDevice(fd_v4l2_, capture_device_path_)) {
        std::cerr << "Failed to open V4L2 capture device: " << capture_device_path_ << std::endl;
        return false;
    }

    if (!PrintCapabilities(fd_v4l2_)) {
        std::cerr << "Failed to print device capabilities" << std::endl;
        return false;
    }

    if (!SetFormat(fd_v4l2_)) {
        std::cerr << "Failed to set V4L2 format" << std::endl;
        return false;
    }

    if (!InitV4L2Buffers(fd_v4l2_, buffers_, n_buffers_)) {
        std::cerr << "Failed to initialize V4L2 buffers" << std::endl;
        return false;
    }

    if (!StartStreaming(fd_v4l2_)) {
        std::cerr << "Failed to start V4L2 streaming" << std::endl;
        return false;
    }

    if (!OpenDevice(fd_timestamp_, timestamp_device_path_)) {
        std::cerr << "Failed to open V4L2 timestamp device: " << timestamp_device_path_ << std::endl;
        return false;
    }

    if (!PrintCapabilities(fd_timestamp_) ||
        !SetTimestampFormat(fd_timestamp_) ||
        !InitV4L2Buffers(fd_timestamp_, timestamp_buffers_, n_timestamp_buffers_) ||
        !StartStreaming(fd_timestamp_)) {
        std::cerr << "Failed to initialize V4L2 timestamp device: "
                  << timestamp_device_path_ << std::endl;
        return false;
    }

    return true;
}

bool V4L2Capture::StartCapture(const std::string& output_dir) {
    if (is_capturing_.load()) {
        std::cerr << "Already capturing!" << std::endl;
        return false;
    }

    if (!EnsureDirectory(output_dir)) {
        std::cerr << "Failed to create output directory: " << output_dir << std::endl;
        return false;
    }

    output_dir_ = output_dir;

    if (!InitMppEncoder()) {
        std::cerr << "Failed to initialize MPP encoder" << std::endl;
        return false;
    }

    is_capturing_ = true;
    capture_thread_ = std::thread(&V4L2Capture::CaptureLoop, this);

    std::cout << "Started capturing JPEG frames into: " << output_dir_ << std::endl;
    return true;
}

void V4L2Capture::StopCapture() {
    if (!is_capturing_.load()) {
        return;
    }

    is_capturing_ = false;

    if (capture_thread_.joinable()) {
        capture_thread_.join();
    }

    // 清理MPP资源
    if (mpp_frm_) {
        mpp_frame_deinit(&mpp_frm_);
        mpp_frm_ = nullptr;
    }
    if (mpp_pkt_) {
        mpp_packet_deinit(&mpp_pkt_);
        mpp_pkt_ = nullptr;
    }
    if (mpp_enc_) {
        mpp_destroy(mpp_enc_);
        mpp_enc_ = nullptr;
    }
}

bool V4L2Capture::OpenDevice(int& fd, const std::string& device_path) {
    fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK, 0);
    if (fd == -1) {
        perror("Opening video device");
        return false;
    }

    return true;
}

bool V4L2Capture::PrintCapabilities(int fd) {
    struct v4l2_capability cap;
    if (ioctl(fd, VIDIOC_QUERYCAP, &cap) < 0) {
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

bool V4L2Capture::SetFormat(int fd) {
    // 首先尝试查询设备支持的所有格式
    struct v4l2_fmtdesc fmtdesc = {};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmtdesc.index = 0;
    
    std::vector<uint32_t> supported_formats;
    while (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) == 0) {
        supported_formats.push_back(fmtdesc.pixelformat);
        std::cout << "Supported format [" << fmtdesc.index << "]: ";
        // 打印像素格式
        char fmt_str[5] = {0};
        memcpy(fmt_str, &fmtdesc.pixelformat, 4);
        std::cout << fmt_str << " (" << fmtdesc.description << ")" << std::endl;
        
        fmtdesc.index++;
    }
    
    //Although RK3588 supports NV12 Multiplane, but it seems only 1 plane is used!!!

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = width_;
    fmt.fmt.pix_mp.height = height_;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    fmt.fmt.pix_mp.pixelformat = input_format_; //force to use this NV12
    fmt.fmt.pix_mp.num_planes = 1;
    #if 0
    // can't set 2 plane as genera NV12 format, driver will handle that automatically
    fmt.fmt.pix_mp.plane_fmt[0].sizeimage = width_ * height_; //Y plane size
    fmt.fmt.pix_mp.plane_fmt[0].bytesperline = width_;
    fmt.fmt.pix_mp.plane_fmt[1].sizeimage = width_ * height_ / 2;
    fmt.fmt.pix_mp.plane_fmt[1].bytesperline = width_;
    #endif
    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) 
    {
        perror("Setting Pixel Format to NV12 failed");
        return false;
    }

    // 更新实际的宽高（驱动可能会调整参数）
    width_ = fmt.fmt.pix_mp.width;
    height_ = fmt.fmt.pix_mp.height;

    std::cout << "Set format to " << width_ << "x" << height_ << " with pixel format: ";
    // 打印实际使用的格式
    char fmt_str[5] = {0};
    memcpy(fmt_str, &input_format_, 4);
    std::cout << fmt_str << std::endl;
    std::cout << "Actual planes: " << static_cast<int>(fmt.fmt.pix_mp.num_planes) << std::endl;
    std::cout << "Plane 0 size: " << fmt.fmt.pix_mp.plane_fmt[0].sizeimage << std::endl;
    return true;
}

bool V4L2Capture::SetTimestampFormat(int fd) {
    struct v4l2_fmtdesc fmtdesc = {};
    fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmtdesc.index = 0;

    if (ioctl(fd, VIDIOC_ENUM_FMT, &fmtdesc) < 0) {
        perror("Enumerating timestamp device formats");
        return false;
    }

    struct v4l2_format fmt = {};
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    fmt.fmt.pix_mp.width = width_;
    fmt.fmt.pix_mp.height = height_;
    fmt.fmt.pix_mp.field = V4L2_FIELD_ANY;
    fmt.fmt.pix_mp.pixelformat = fmtdesc.pixelformat;
    fmt.fmt.pix_mp.num_planes = 1;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("Setting timestamp device format failed");
        return false;
    }

    char fmt_str[5] = {0};
    memcpy(fmt_str, &fmt.fmt.pix_mp.pixelformat, 4);
    std::cout << "Timestamp format set to "
              << fmt.fmt.pix_mp.width << "x" << fmt.fmt.pix_mp.height
              << " with pixel format: " << fmt_str << std::endl;
    std::cout << "Timestamp plane 0 size: "
              << fmt.fmt.pix_mp.plane_fmt[0].sizeimage << std::endl;
    return true;
}

bool V4L2Capture::InitV4L2Buffers(int fd, buffer*& buffers, int& n_buffers) {
    struct v4l2_requestbuffers req = {};
    req.count = n_buffers_;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("Requesting buffer");
        return false;
    }
    else
    {
        std::cout << "Requested " << req.count << " buffers successfully" << std::endl;
    }

    n_buffers = req.count;
    buffers = new buffer[n_buffers];

    for (uint32_t i = 0; i < req.count; ++i) 
    {
        struct v4l2_plane planes[VIDEO_MAX_PLANES] = {}; // 1 plane for RK3588 NV12
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;
        buf.m.planes = planes;
        buf.length = 1; // 1 plane for NV12

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("Query buffer for MPLANE");
            return false;
        }

        std::cout << "Buffer " << i << ":" << std::endl;
        std::cout << "  NV12 Plane Offset: " << buf.m.planes[0].m.mem_offset << std::endl;
        std::cout << "  NV12 Plane Length: " << buf.m.planes[0].length << std::endl;

        // 对于多平面缓冲区，我们需要映射总的内存大小
        // NV12格式通常有两个平面：Y平面和UV平面，但RK3588这里只有一个
        buffers[i].length = buf.m.planes[0].length;
        buffers[i].start = mmap(nullptr, buffers[i].length, 
                            PROT_READ | PROT_WRITE, MAP_SHARED, 
                    fd, buf.m.planes[0].m.mem_offset);
        
        if (buffers[i].start == MAP_FAILED) {
            perror("Mapping buffer for MPLANE");
            return false;
        }

         // 将所有缓冲区加入队列
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            perror("Queuing buffer for MPLANE");
            return false;
        }
    }
    return true;
}

bool V4L2Capture::StartStreaming(int fd) {
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        perror("Start streaming");
        return false;
    }
    else
    {
        std::cout << "Started streaming successfully" << std::endl;
    }
    return true;
}

bool V4L2Capture::StopStreaming(int fd) {
    enum v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    
    if (ioctl(fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("Stop streaming");
        return false;
    }
    return true;
}

void V4L2Capture::CloseDevice() {
    CloseDevice(fd_v4l2_, buffers_, n_buffers_);
    CloseDevice(fd_timestamp_, timestamp_buffers_, n_timestamp_buffers_);
}

void V4L2Capture::CloseDevice(int& fd, buffer*& buffers, int n_buffers) {
    if (buffers) {
        for (int i = 0; i < n_buffers; ++i) {
            if (buffers[i].start != MAP_FAILED) {
                munmap(buffers[i].start, buffers[i].length);
            }
        }
        delete[] buffers;
        buffers = nullptr;
    }

    if (fd != -1) {
        close(fd);
        fd = -1;
    }
}

void V4L2Capture::CaptureLoop() {
    uint32_t frame_count = 1;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto tpNow = std::chrono::system_clock::now();
    std::time_t tNow = std::chrono::system_clock::to_time_t(tpNow);
    firstPts_ = tNow * 1000;

    while (is_capturing_.load()) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(fd_v4l2_, &fds);
        FD_SET(fd_timestamp_, &fds);

        struct timeval tv;
        tv.tv_sec = 2;
        tv.tv_usec = 0;

        int max_fd = std::max(fd_v4l2_, fd_timestamp_);
        int r = select(max_fd + 1, &fds, nullptr, nullptr, &tv);
        if (r == -1) {
            perror("Select failed");
            break;
        } else if (r == 0) {
            std::cerr << "Select timeout" << std::endl;
            continue;
        }

        if (!FD_ISSET(fd_v4l2_, &fds) || !FD_ISSET(fd_timestamp_, &fds)) {
            continue;
        }

        struct v4l2_buffer timestamp_buf = {};
        struct v4l2_plane timestamp_planes[VIDEO_MAX_PLANES] = {};
        timestamp_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        timestamp_buf.memory = V4L2_MEMORY_MMAP;
        timestamp_buf.m.planes = timestamp_planes;
        timestamp_buf.length = 1;

        if (ioctl(fd_timestamp_, VIDIOC_DQBUF, &timestamp_buf) < 0) {
            perror("Retrieving timestamp frame");
            break;
        }

        struct v4l2_buffer capture_buf = {};
        struct v4l2_plane capture_planes[VIDEO_MAX_PLANES] = {};
        capture_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
        capture_buf.memory = V4L2_MEMORY_MMAP;
        capture_buf.m.planes = capture_planes;
        capture_buf.length = 1;

        if (ioctl(fd_v4l2_, VIDIOC_DQBUF, &capture_buf) < 0) {
            perror("Retrieving capture frame");
            timestamp_buf.m.planes = timestamp_planes;
            ioctl(fd_timestamp_, VIDIOC_QBUF, &timestamp_buf);
            break;
        }

        // The encoded image comes from the capture device; its timestamp comes from the reference device.
        struct timeval v4l2_timestamp = timestamp_buf.timestamp;
        int64_t v4l2_pts_us = (int64_t)v4l2_timestamp.tv_sec * 1000000LL + (int64_t)v4l2_timestamp.tv_usec;
        if (v4l2_pts_us <= 0) {
            v4l2_pts_us = frame_count;
        }

        // 使用MPP编码帧并保存为JPEG文件
        if (!EncodeFrameWithMpp(capture_buf, v4l2_pts_us)) {
            std::cerr << "Failed to encode frame with MPP" << std::endl;
        }

        // 将缓冲区重新排队
        if (ioctl(fd_v4l2_, VIDIOC_QBUF, &capture_buf) < 0) {
            perror("Requeuing buffer");
            break;
        }

        if (ioctl(fd_timestamp_, VIDIOC_QBUF, &timestamp_buf) < 0) {
            perror("Requeuing timestamp buffer");
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

    // 将 x 向上对齐到 a 的倍数（a 必须是 2 的幂，如 16, 32, 64）
#define ALIGN_UP(x, a) (((x) + ((a) - 1)) & ~((a) - 1))
// 新增MPP编码器初始化函数
bool V4L2Capture::InitMppEncoder() {
    MPP_RET ret;

    // 创建MPP编码器上下文
    ret = mpp_create(&mpp_enc_, &mpp_api_);
    if (ret != MPP_OK) {
        std::cerr << "Failed to create MPP encoder: " << ret << std::endl;
        return false;
    }
    // 将配置应用到MPP编码器
    ret = mpp_init(mpp_enc_, MPP_CTX_ENC, MPP_VIDEO_CodingMJPEG);
    if (ret != MPP_OK) {
        std::cerr << "Failed to init MPP encoder: " << ret << std::endl;
        return false;
    }

    // 配置编码器为JPEG模式
    MppEncCfg enc_cfg = nullptr;
    ret = mpp_enc_cfg_init(&enc_cfg);
    if (ret != MPP_OK) {
        std::cerr << "Failed to init MPP encoder config: " << ret << std::endl;
        return false;
    }

    // 设置编码器配置
    mpp_enc_cfg_set_s32(enc_cfg, "prep:format", MPP_FMT_YUV420SP); // NV12格式
    mpp_enc_cfg_set_s32(enc_cfg, "prep:width", width_);
    mpp_enc_cfg_set_s32(enc_cfg, "prep:height", height_);
    mpp_enc_cfg_set_s32(enc_cfg, "jpeg:q_factor", 80); // JPEG质量因子
    // 设置码率控制参数
    //mpp_enc_cfg_set_s32(enc_cfg, "rc:mode", MPP_ENC_RC_MODE_CBR);
    //mpp_enc_cfg_set_s32(enc_cfg, "rc:bps_target", 10000000); // 目标码率 10Mbps



        // 假设 ALIGN_16 是一个将数值向上对齐到 16 的宏
    mpp_enc_cfg_set_s32(enc_cfg, "prep:hor_stride", ALIGN_UP(width_, 16));
    mpp_enc_cfg_set_s32(enc_cfg, "prep:ver_stride", ALIGN_UP(height_, 16));

    // 4. 将配置应用到编码器
    ret = mpp_api_->control(mpp_enc_, MPP_ENC_SET_CFG, enc_cfg);
    if (ret != MPP_OK) {
        std::cerr << "Failed to mpp_api_->control: " << ret << std::endl;
        return false;
    }

    // 销毁临时配置
    mpp_enc_cfg_deinit(enc_cfg);

    // 初始化MPP帧对象
    ret = mpp_frame_init(&mpp_frm_);
    if (ret != MPP_OK) {
        std::cerr << "Failed to init MPP frame: " << ret << std::endl;
        return false;
    }

    // 设置帧参数
    mpp_frame_set_width(mpp_frm_, width_);
    mpp_frame_set_height(mpp_frm_, height_);
    mpp_frame_set_fmt(mpp_frm_, MPP_FMT_YUV420SP); // NV12格式

    return true;
}

// 新增使用MPP编码帧的函数
bool V4L2Capture::EncodeFrameWithMpp(struct v4l2_buffer& buf, int64_t timestamp_us) {
    MPP_RET ret;

    // 准备MPP帧数据
    size_t frame_size = width_ * height_ * 3 / 2; // NV12格式大小

    // 创建MPP缓冲区
    MppBuffer frm_buf = NULL;
    ret = mpp_buffer_get(NULL, &frm_buf, frame_size);
    if (ret != MPP_OK) {
        std::cerr << "Failed to get MPP buffer: " << ret << std::endl;
        return false;
    }

    // 复制V4L2缓冲区数据到MPP缓冲区
    void* mpp_data = mpp_buffer_get_ptr(frm_buf);
    memcpy(mpp_data, buffers_[buf.index].start, frame_size);

    // 设置帧的缓冲区
    mpp_frame_set_buffer(mpp_frm_, frm_buf);
    mpp_frame_set_width(mpp_frm_, width_);
    mpp_frame_set_height(mpp_frm_, height_);
    mpp_frame_set_fmt(mpp_frm_, MPP_FMT_YUV420SP);
    mpp_frame_set_pts(mpp_frm_, timestamp_us);

    // 发送帧到编码器 - 使用标准MPP API
    ret = mpp_api_->encode_put_frame(mpp_enc_, mpp_frm_);
    if (ret != MPP_OK) {
        std::cerr << "Failed to put frame to MPP encoder: " << ret << std::endl;
        mpp_buffer_put(frm_buf);
        return false;
    }

    // 从编码器接收编码后的数据包 - 使用正确的API
    MppPacket packet = NULL;
    ret = mpp_api_->encode_get_packet(mpp_enc_, &packet);
    if (ret != MPP_OK) {
        std::cerr << "Failed to get encoded packet from MPP: " << ret << std::endl;
        mpp_buffer_put(frm_buf);
        return false;
    }
#if 1
    if (packet) {
        // 写入JPEG文件
        std::ostringstream filename;
        filename << output_dir_ << "/" << timestamp_us << ".jpg";

        FILE* fp = fopen(filename.str().c_str(), "wb");
        if (!fp) {
            perror("Opening JPEG output file");
            mpp_buffer_put(frm_buf);
            if (packet) mpp_packet_deinit(&packet);
            return false;
        }

        size_t data_size = mpp_packet_get_length(packet);
        void* data = mpp_packet_get_pos(packet);

        if (fwrite(data, 1, data_size, fp) != data_size) {
            perror("Writing JPEG output file");
            fclose(fp);
            mpp_buffer_put(frm_buf);
            if (packet) mpp_packet_deinit(&packet);
            return false;
        }

        fclose(fp);
        //std::cout << "Saved JPEG frame: " << filename.str() << " (size: " << data_size << " bytes)" << std::endl;
    }
#endif
    // 释放资源
    mpp_buffer_put(frm_buf);
    if (packet) mpp_packet_deinit(&packet);

    return true;
}


bool V4L2Capture::EnsureDirectory(const std::string& dir_path) {
    if (dir_path.empty()) {
        return true;
    }

    if (mkdir(dir_path.c_str(), 0755) == 0) {
        return true;
    }

    if (errno == EEXIST) {
        return true;
    }

    return false;
}

// 为了兼容性保留这些函数，但不使用它们
bool V4L2Capture::GetFrameData(AVFrame*& frame) {
    return false;
}

void V4L2Capture::ReleaseFrame(int buffer_idx) {
}

// 新增兼容性函数
bool V4L2Capture::InitEncoder() {
    return true;
}

bool V4L2Capture::EncodeFrame(AVFrame* frame, int64_t timestamp_us) {
    return true;
}

bool V4L2Capture::ConvertToYUV420P(uint8_t* src_data, AVFrame* dst_frame) {
    return true;
}