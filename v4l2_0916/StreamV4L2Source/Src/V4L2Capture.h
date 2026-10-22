#pragma once

#include <string>
#include <thread>
#include <atomic>
#include "VideoEncoder.h"
#include "StreamV4L2Source.h"
#include "V4L2StreamCallback.h"

extern "C" {
#include <libavformat/avformat.h>
#include <libavcodec/avcodec.h>
#include <libswscale/swscale.h>
#include <libavutil/imgutils.h>
#include <linux/videodev2.h>
}

namespace V4L2StreamLib
{
	class V4L2Capture
	{
	public:
		struct buffer {
			void* start;
			size_t length;
		};
		V4L2Capture(const std::string& device_path);
		~V4L2Capture();

		bool Initialize(const VideoParams& param);
		void UnInit();
		void SetStreamCallback(StreamFrameCallback callback, void* userData);
		void SetStreamCallback(V4L2FrameCallback callback, void* userData);

		bool StartCapture();
		void StopCapture();

		bool GetFrameData(AVFrame*& frame);
		void ReleaseFrame(int buffer_idx);
	private:
		bool OpenDevice();

		bool PrintCapabilities();
		bool SetFormat();
		bool InitV4L2Buffers();
		bool StartStreaming();
		bool StopStreaming();

		void CloseDevice();
		void CaptureLoop();

	private:
		std::string device_path_;
		int fd_v4l2_ = -1;
		buffer* buffers_ = nullptr;
		int n_buffers_ = 4;
		uint32_t input_format_ = V4L2_PIX_FMT_NV12;

		std::thread capture_thread_;
		std::atomic_bool is_capturing_ = false;

		// 添加缓冲区索引跟踪
		std::atomic<int> current_buffer_idx_ = -1;

		VideoEncoder videoEncoder_;
		VideoParams encodeParams_;
		V4L2FrameCallback frameCallback_ = nullptr;
		void* frameCallbackParam_ = nullptr;
		int64_t firstPts_ = 0;
		int64_t firstV4L2_pts_us_ = 0;
	};
}