#pragma once
#include <vector>
#include <string>
#include <mutex>
#include <memory>
#include "StreamCommon.h"
#include "V4L2StreamCallback.h"
namespace V4L2StreamLib
{
	class StreamV4L2SourceImpl;
}

struct VideoParams
{
	int codec_id = 27;//AV_CODEC_ID_H264

	int width = 1280;
	int height = 720;
	int bit_rate = 2000000;
	int framerate = 25;
	int gop_size = 25;
	int is_compressed = 0;
};

class StreamV4L2Source
{
public:
	StreamV4L2Source();
	~StreamV4L2Source();

	bool Init(const VideoParams& param, const std::vector<std::string>& deviceList);

	bool StartStream(StreamFrameCallback callback, void* userData);
	bool StartStream(V4L2FrameCallback callback, void* userData);
	bool StartFrame(AVFrame* frame, void* userData);
	bool StopStream();

private:
	std::unique_ptr<V4L2StreamLib::StreamV4L2SourceImpl> upStream_;
};

