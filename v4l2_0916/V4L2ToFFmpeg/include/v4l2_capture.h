#ifndef V4L2_CAPTURE_H
#define V4L2_CAPTURE_H

#include <string>
#include <thread>
#include <atomic>
#include <linux/videodev2.h>

#ifdef __cplusplus
extern "C" {
#endif
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
// 包含MPP库头文件
#include <rockchip/rk_mpi.h>
#include <rockchip/rk_type.h>
#include <rockchip/mpp_frame.h>
#include <rockchip/mpp_buffer.h>
#include <rockchip/mpp_packet.h>
#ifdef __cplusplus
}
#endif

struct buffer {
    void* start;
    size_t length;
};

class V4L2Capture {
public:
    V4L2Capture(const std::string& timestamp_device_path,
                const std::string& capture_device_path);
    ~V4L2Capture();

    bool Initialize(int width, int height, int fps);
    bool StartCapture(const std::string& output_dir);
    void StopCapture();

private:
    // V4L2相关成员
    std::string timestamp_device_path_;
    std::string capture_device_path_;
    int fd_v4l2_;
    buffer* buffers_;
    int n_buffers_;
    int fd_timestamp_;
    buffer* timestamp_buffers_;
    int n_timestamp_buffers_;
    int width_, height_, fps_;
    uint32_t input_format_;
    std::string output_dir_;
    
    // MPP相关成员变量
    MppCtx mpp_enc_;
    MppApi* mpp_api_;
    MppEncCfg mpp_enc_cfg_;
    MppFrame mpp_frm_;
    MppPacket mpp_pkt_;
    
    // 其他成员
    std::thread capture_thread_;
    std::atomic<bool> is_capturing_;
    int64_t firstPts_;

    // V4L2操作函数
    bool OpenDevice(int& fd, const std::string& device_path);
    bool PrintCapabilities(int fd);
    bool SetFormat(int fd);
    bool SetTimestampFormat(int fd);
    bool InitV4L2Buffers(int fd, buffer*& buffers, int& n_buffers);
    bool StartStreaming(int fd);
    bool StopStreaming(int fd);
    void CloseDevice();
    void CloseDevice(int& fd, buffer*& buffers, int n_buffers);
    
    // MPP编码器相关函数
    bool InitMppEncoder();
    bool EncodeFrameWithMpp(struct v4l2_buffer& buf, int64_t timestamp_us);
    
    // 旧的FFmpeg编码器相关函数（保留但不使用）
    bool InitEncoder(); // 保留为了兼容性
    bool EncodeFrame(AVFrame* frame, int64_t timestamp_us); // 保留为了兼容性
    
    // 辅助函数
    void CaptureLoop();
    bool ConvertToYUV420P(uint8_t* src_data, AVFrame* dst_frame);
    bool EnsureDirectory(const std::string& dir_path);
    
    // 新增函数
    bool GetFrameData(AVFrame*& frame);
    void ReleaseFrame(int buffer_idx);
};

#endif // V4L2_CAPTURE_H