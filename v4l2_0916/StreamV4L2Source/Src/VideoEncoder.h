#pragma once

#include "ffmpeg.h"
#include "StreamCommon.h"
#include "StreamV4L2Source.h"

namespace V4L2StreamLib
{
	class VideoEncoder
	{
	public:
		VideoEncoder();
		~VideoEncoder();

		bool InitEncoder(const VideoParams& params);
		void Close();
		void SetStreamCallback(StreamFrameCallback callback, void* userData);

		bool OnFrame(void* pData, AVFrame* pFrame);

	private:
		bool m_bInited = false;
		const AVCodec* pEncodec_ = nullptr;
		AVCodecContext* pEncodeContext_ = nullptr;
		AVDictionary* pdict_ = nullptr;
		StreamFrameCallback callback_ = nullptr;
		void* userData_ = nullptr;
		int64_t pts_ = 0;
	};
}