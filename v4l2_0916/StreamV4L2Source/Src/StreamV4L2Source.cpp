#include "StreamV4L2Source.h"
#include "StreamV4L2SourceImpl.h"

StreamV4L2Source::StreamV4L2Source()
{
	upStream_ = std::make_unique<V4L2StreamLib::StreamV4L2SourceImpl>();
}

StreamV4L2Source::~StreamV4L2Source()
{

}

bool StreamV4L2Source::Init(const VideoParams& param, const std::vector<std::string>& deviceList)
{
	return upStream_->Initialize(param, deviceList);
}

bool StreamV4L2Source::StartStream(StreamFrameCallback callback, void* userData)
{
	return upStream_->StartStream(callback, userData);
}

bool StreamV4L2Source::StartStream(V4L2FrameCallback callback, void* userData)
{
	return upStream_->StartStream(callback, userData);
}

bool StreamV4L2Source::StopStream()
{
	return upStream_->StopStream();
}
