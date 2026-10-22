#include "DualV4L2Stitcher.h"
#include <iostream>
#include <thread>

namespace V4L2StreamLib
{

	DualV4L2Stitcher::DualV4L2Stitcher(const std::vector<std::string>& deviceList)
		: capture1_(deviceList[0])
		, capture2_(deviceList[1])
	{

	}

	DualV4L2Stitcher::~DualV4L2Stitcher()
	{

	}

	bool DualV4L2Stitcher::Initialize(const VideoParams& param)
	{
		if (!capture1_.Initialize(param) || !capture2_.Initialize(param)) {
			std::cerr << "Failed to initialize V4L2 capture devices" << std::endl;
			return false;
		}

		if (!InitFiltersAndEncoder()) {
			std::cerr << "Failed to initialize filters and encoder" << std::endl;
			return false;
		}

		return true;
	}

	void DualV4L2Stitcher::UnInit()
	{
		capture1_.UnInit();
		capture2_.UnInit();
		UnInitFiltersAndEncoder();
	}

	void DualV4L2Stitcher::SetStreamCallback(StreamFrameCallback callback, void* userData)
	{
		videoEncoder_.SetStreamCallback(callback, userData);
	}

	void DualV4L2Stitcher::SetStreamCallback(V4L2FrameCallback callback, void* userData)
	{
		frameCallback_ = callback;
		frameCallbackParam_ = userData;
	}

	bool DualV4L2Stitcher::InitFiltersAndEncoder() {
		bool ret = InitFilters();
		if (ret)
			ret = InitEncoder();
		if (!ret)
		{

		}
		return ret;
	}

	bool DualV4L2Stitcher::InitFilters()
	{
		// 初始化硬件设备上下文，相当于命令行中的 "-init_hw_device rkmpp=hw"
		int ret = av_hwdevice_ctx_create(&hw_device_ctx_, av_hwdevice_find_type_by_name("rkmpp"), "rkmpp", NULL, 0);
		if (ret < 0) {
			std::cout << "Failed to create rkmpp hardware device context, continuing without explicit hw context..." << std::endl;
			// 这不是致命错误，因为overlay_rkrga和h264_rkmpp有自己独立的资源管理
		}
		else {
			std::cout << "Successfully created rkmpp hardware device context" << std::endl;
		}

		// 创建滤镜图
		filter_graph_ = avfilter_graph_alloc();
		if (!filter_graph_) {
			std::cerr << "Could not allocate filter graph" << std::endl;
			return false;
		}

		// 将硬件设备上下文关联到滤镜图，相当于命令行中的 "-filter_hw_device hw"
		if (hw_device_ctx_) {
			// 方法1: 使用av_opt_set将硬件设备上下文关联到滤镜图
			ret = av_opt_set(filter_graph_, "hwaccel_device", "rkmpp", 0);
			if (ret < 0) {
				std::cerr << "Failed to set hardware device for filter graph" << std::endl;
			}
		}

		// 创建输入滤镜
		char args1[512];
		snprintf(args1, sizeof(args1),
			"video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
			encodeParams_.width, encodeParams_.height, AV_PIX_FMT_NV12, 1, encodeParams_.framerate, 1, 1);

		ret = avfilter_graph_create_filter(&src1_ctx_, avfilter_get_by_name("buffer"), "src1",
			args1, NULL, filter_graph_);
		if (ret < 0) {
			std::cerr << "Cannot create buffer source 1" << std::endl;
			return false;
		}

		char args2[512];
		snprintf(args2, sizeof(args2),
			"video_size=%dx%d:pix_fmt=%d:time_base=%d/%d:pixel_aspect=%d/%d",
			encodeParams_.width, encodeParams_.height, AV_PIX_FMT_NV12, 1, encodeParams_.framerate, 1, 1);

		ret = avfilter_graph_create_filter(&src2_ctx_, avfilter_get_by_name("buffer"), "src2",
			args2, NULL, filter_graph_);
		if (ret < 0) {
			std::cerr << "Cannot create buffer source 2" << std::endl;
			return false;
		}

		// 创建输出滤镜
		ret = avfilter_graph_create_filter(&sink_ctx_, avfilter_get_by_name("buffersink"), "out",
			NULL, NULL, filter_graph_);
		if (ret < 0) {
			std::cerr << "Cannot create buffer sink" << std::endl;
			return false;
		}

		// 设置输出格式
		enum AVPixelFormat pix_fmts[] = { AV_PIX_FMT_NV12, AV_PIX_FMT_NONE };
		ret = av_opt_set_int_list(sink_ctx_, "pix_fmts", pix_fmts, AV_PIX_FMT_NONE, AV_OPT_SEARCH_CHILDREN);
		if (ret < 0) {
			std::cerr << "Cannot set output pixel format" << std::endl;
			return false;
		}

		// 构建滤镜链 - 使用hstack滤镜进行水平拼接
		AVFilterInOut* inputs = nullptr;
		AVFilterInOut* outputs = nullptr;

		// 使用hstack滤镜进行水平拼接，将两个相同大小的视频并排组合
		std::string filter_descr = "[0]format=nv12,setpts=PTS-STARTPTS[vid0];";
		filter_descr += "[1]format=nv12,setpts=PTS-STARTPTS[vid1];";
		filter_descr += "[vid0][vid1]hstack[out]";
		std::cout << "Using hstack filter parameters: " + filter_descr << std::endl;

		ret = avfilter_graph_parse2(filter_graph_, filter_descr.c_str(), &inputs, &outputs);
		if (ret < 0)
		{
			std::cerr << "hstack filter failed, exit " << ret << std::endl;
			return false;
		}

		// 手动连接输入源到滤镜图
		int input_index = 0;
		for (AVFilterInOut* current_input = inputs; current_input; current_input = current_input->next, ++input_index) {
			AVFilterContext* src_ctx = (input_index == 0) ? src1_ctx_ : src2_ctx_;
			ret = avfilter_link(src_ctx, 0, current_input->filter_ctx, current_input->pad_idx);
			if (ret < 0) {
				std::cerr << "Failed to connect input filter" << std::endl;
				avfilter_inout_free(&inputs);
				avfilter_inout_free(&outputs);
				return false;
			}
			else {
				std::cout << "Input buffer " << input_index << " connected" << std::endl;
			}
		}

		// 连接滤镜图输出到sink
		for (AVFilterInOut* output = outputs; output; output = output->next) {
			ret = avfilter_link(output->filter_ctx, output->pad_idx, sink_ctx_, 0);
			if (ret < 0) {
				std::cerr << "Failed to connect output filter" << std::endl;
				avfilter_inout_free(&inputs);
				avfilter_inout_free(&outputs);
				return false;
			}
			else {
				std::cout << "Output buffersink connected" << std::endl;
			}
		}

		avfilter_inout_free(&inputs);
		avfilter_inout_free(&outputs);

		// 配置滤镜图
		ret = avfilter_graph_config(filter_graph_, NULL);
		if (ret < 0) {
			std::cerr << "Error configuring filter graph" << std::endl;
			return false;
		}

		char* graph = avfilter_graph_dump(filter_graph_, nullptr);
		if (graph) {
			std::cout << "Filter graph configuration:\n" << graph << std::endl;
			av_free(graph);
		}
		else {
			std::cerr << "Failed to dump null filter graph" << std::endl;
		}
		return true;
	}

	bool DualV4L2Stitcher::InitEncoder()
	{
		VideoParams encodeParams = encodeParams_;
		encodeParams.width *= 2;
		encodeParams.bit_rate *= 2;
		bool ret = videoEncoder_.InitEncoder(encodeParams);
		return ret;
	}

	void DualV4L2Stitcher::UnInitFiltersAndEncoder()
	{
		if (filter_graph_) {
			avfilter_graph_free(&filter_graph_);
		}

		if (hw_device_ctx_) {
			av_buffer_unref(&hw_device_ctx_);
		}
		videoEncoder_.Close();
	}

	bool DualV4L2Stitcher::StartCapture()
	{
		if (is_capturing_.load()) {
			std::cerr << "Already capturing!" << std::endl;
		}
		is_capturing_ = true;
		capture_thread_ = std::thread(&DualV4L2Stitcher::ProcessFrames, this);
		return true;
	}

	void DualV4L2Stitcher::StopCapture()
	{
		if (!is_capturing_.load()) {
			return;
		}

		is_capturing_ = false;

		if (capture_thread_.joinable()) {
			capture_thread_.join();
		}
	}


	void DualV4L2Stitcher::ProcessFrames()
	{
		std::cout << "Starting dual capture and stitching..." << std::endl;

		// 启动两个摄像头捕获（但我们实际上会直接获取帧）
		// capture1.StartCapture("/tmp/temp1.mp4") 和 capture2.StartCapture("/tmp/temp2.mp4") 
		// 不在这里启动，因为我们直接获取帧

		int frame_count = 0;
		auto start_time = std::chrono::high_resolution_clock::now();

		while (is_capturing_) {
			// 从两个摄像头获取帧
			AVFrame* frame1 = nullptr, * frame2 = nullptr;

			bool got_frame1 = capture1_.GetFrameData(frame1);
			bool got_frame2 = capture2_.GetFrameData(frame2);

			if (got_frame1 && got_frame2) {
				// 将帧发送到滤镜
				int ret1 = av_buffersrc_add_frame(src1_ctx_, frame1);
				int ret2 = av_buffersrc_add_frame(src2_ctx_, frame2);

				if (ret1 >= 0 && ret2 >= 0) {
					// 从滤镜获取拼接后的帧
					AVFrame* filtered_frame = av_frame_alloc();

					ret1 = av_buffersink_get_frame(sink_ctx_, filtered_frame);
					if (ret1 >= 0) {
						// 设置时间戳
						auto current_time = std::chrono::high_resolution_clock::now();
						auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
							current_time - start_time).count();
						filtered_frame->pts = elapsed_us/1000;

						if (encodeParams_.is_compressed == 1)
						{						// 编码并写入文件
							// 编码并写入文件
							if (!EncodeFrame(filtered_frame)) {
								std::cerr << "Failed to encode frame" << std::endl;
							}
						}
						else
						{
							if (frameCallback_)
								frameCallback_(0, filtered_frame, 0, frameCallbackParam_); //TODO, set right capture timestamp
						}


						frame_count++;
						if (frame_count % 30 == 0) {
							std::cout << "Processed " << frame_count << " frames" << std::endl;
						}
					}

					av_frame_free(&filtered_frame);
				}

				// 释放帧
				if (frame1) {
					int idx1 = static_cast<int>(reinterpret_cast<intptr_t>(frame1->opaque));
					capture1_.ReleaseFrame(idx1);
					av_frame_free(&frame1);
				}

				if (frame2) {
					int idx2 = static_cast<int>(reinterpret_cast<intptr_t>(frame2->opaque));
					capture2_.ReleaseFrame(idx2);
					av_frame_free(&frame2);
				}
			}
			else {
				// 如果没有获取到帧，短暂休眠
				// 休眠1ms
				std::this_thread::sleep_for(std::chrono::milliseconds(1));
			}
		}
	}

	bool DualV4L2Stitcher::EncodeFrame(AVFrame* frame)
	{
		return videoEncoder_.OnFrame(this, frame);
	}
}