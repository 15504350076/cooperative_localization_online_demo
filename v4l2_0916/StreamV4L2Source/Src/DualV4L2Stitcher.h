#pragma once
#include "V4L2Capture.h"
#include "VideoEncoder.h"
#include "V4L2StreamCallback.h"
#include <thread>
#include <atomic>
#include <vector>

namespace V4L2StreamLib
{
	class DualV4L2Stitcher
	{
	public:
		DualV4L2Stitcher(const std::vector<std::string>& deviceList);
		~DualV4L2Stitcher();

		bool Initialize(const VideoParams& param);
		void UnInit();
		void SetStreamCallback(StreamFrameCallback callback, void* userData);
		void SetStreamCallback(V4L2FrameCallback callback, void* userData);
		bool StartCapture();
		void StopCapture();
	private:
		bool InitFiltersAndEncoder();
		bool InitFilters();
		bool InitEncoder();
		void UnInitFiltersAndEncoder();

		void ProcessFrames();
		bool EncodeFrame(AVFrame* frame);
	private:
		V4L2Capture capture1_;
		V4L2Capture capture2_;

		VideoEncoder videoEncoder_;
		VideoParams encodeParams_;
		V4L2FrameCallback frameCallback_ = nullptr;
		void* frameCallbackParam_ = nullptr;
		std::thread capture_thread_;
		std::atomic_bool is_capturing_ = false;

		// FFmpeg滤镜相关
		AVFilterGraph* filter_graph_ = nullptr;
		AVFilterContext* src1_ctx_ = nullptr;
		AVFilterContext* src2_ctx_ = nullptr;
		AVFilterContext* overlay_ctx_ = nullptr;
		AVFilterContext* sink_ctx_ = nullptr;
		// 硬件设备上下文
		AVBufferRef* hw_device_ctx_ = nullptr;


	};
}