#pragma once
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include "StreamV4L2Source.h"
#include "V4L2StreamCallback.h"
#
namespace V4L2StreamLib
{
	class DualV4L2Stitcher;
	class V4L2Capture;

	class StreamV4L2SourceImpl
	{
	public:
		StreamV4L2SourceImpl();
		~StreamV4L2SourceImpl();

		bool Initialize(const VideoParams& param, const std::vector<std::string>& deviceList);
		void UnInit();

		bool StartStream(StreamFrameCallback callback, void* userData);
		bool StartStream(V4L2FrameCallback callback, void* userData);
		bool StopStream();

	private:
		std::unique_ptr< DualV4L2Stitcher> upStitcher_;
		std::unique_ptr<V4L2Capture> upCapture_;
	};

}