#include "StreamV4L2SourceImpl.h"
#include "DualV4L2Stitcher.h"
#include "V4L2Capture.h"

namespace V4L2StreamLib
{
	StreamV4L2SourceImpl::StreamV4L2SourceImpl()
	{

	}

	StreamV4L2SourceImpl::~StreamV4L2SourceImpl()
	{

	}

	bool StreamV4L2SourceImpl::Initialize(const VideoParams& param, const std::vector<std::string>& deviceList)
	{
		bool ret = false;
		if (deviceList.size() == 1)
		{
			upCapture_ = std::make_unique<V4L2Capture>(deviceList[0]);
			ret = upCapture_->Initialize(param);
		}
		else if (deviceList.size() == 2)
		{
			upStitcher_ = std::make_unique<DualV4L2Stitcher>(deviceList);
			ret = upStitcher_->Initialize(param);
		}
		else
		{
			return false;
		}
		return ret;
	}
	void StreamV4L2SourceImpl::UnInit()
	{
		if (upCapture_)
			upCapture_->UnInit();
		if (upStitcher_)
			upStitcher_->UnInit();
	}

	bool StreamV4L2SourceImpl::StartStream(StreamFrameCallback callback, void* userData)
	{
		if (upCapture_)
		{
			upCapture_->SetStreamCallback(callback, userData);
			return upCapture_->StartCapture();		
		}
		if (upStitcher_)
		{
			upStitcher_->SetStreamCallback(callback, userData);
			upStitcher_->StartCapture();
			return true;
		}
		return false;
	}

	bool StreamV4L2SourceImpl::StartStream(V4L2FrameCallback callback, void* userData)
	{
		if (upCapture_)
		{
			upCapture_->SetStreamCallback(callback, userData);
			return upCapture_->StartCapture();		
		}
		if (upStitcher_)
		{
			upStitcher_->SetStreamCallback(callback, userData);
			upStitcher_->StartCapture();
			return true;
		}
		return false;
	}

	bool StreamV4L2SourceImpl::StopStream()
	{
		if(upCapture_)
		{
			upCapture_->StopCapture();
		}
		if (upStitcher_)
		{
			upStitcher_->StopCapture();
		}
		return true;
	}
}