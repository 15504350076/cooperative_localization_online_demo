#include "VideoEncoder.h"
#include <chrono>
#include <iostream>

namespace V4L2StreamLib
{
	VideoEncoder::VideoEncoder()
	{

	}

	VideoEncoder::~VideoEncoder()
	{

	}

	bool VideoEncoder::InitEncoder(const VideoParams& params)
	{
		pEncodec_ = avcodec_find_encoder_by_name("h264_rkmpp");
		if (!pEncodec_)
		{
			Close();
			return false;
		}

		pEncodeContext_ = avcodec_alloc_context3(pEncodec_);
		if (pEncodeContext_ == nullptr)
		{
			Close();
			return false;
		}
		pEncodeContext_->width = params.width;
		pEncodeContext_->height = params.height;
		//pEncodeContext_->sample_aspect_ratio.den = 45;
		//pEncodeContext_->sample_aspect_ratio.num = 64;
		pEncodeContext_->bit_rate = params.bit_rate; //400k
		pEncodeContext_->rc_max_rate = params.bit_rate;
		pEncodeContext_->gop_size = params.gop_size;
		pEncodeContext_->has_b_frames = false;
		pEncodeContext_->max_b_frames = 0;
		pEncodeContext_->refs = 1;

		pEncodeContext_->time_base.num = 1;
		pEncodeContext_->time_base.den = params.framerate;
		pEncodeContext_->framerate.num = params.framerate;
		pEncodeContext_->framerate.den = 1;
		pEncodeContext_->flags |= AV_CODEC_FLAG_LOW_DELAY;
		pEncodeContext_->codec_type = AVMEDIA_TYPE_VIDEO;

		pEncodeContext_->codec_id = AV_CODEC_ID_H264;
		pEncodeContext_->pix_fmt = AV_PIX_FMT_NV12;

		av_dict_set(&pdict_, "preset", "fast", 0);
		av_dict_set(&pdict_, "tune", "zerolatency", 0);

		int nResult = avcodec_open2(pEncodeContext_, pEncodec_, &pdict_);
		if (nResult < 0)
		{
			std::cout << "avcodec_open2 failed, error code: " << nResult << std::endl;
			Close();
			return false;
		}
		pts_ = 0;

		m_bInited = true;
		return true;
	}

	void VideoEncoder::Close()
	{
		if (pEncodeContext_ != nullptr)
		{
			pEncodeContext_->extradata_size = 0;
			pEncodeContext_->extradata = nullptr;
			avcodec_close(pEncodeContext_);
			avcodec_free_context(&pEncodeContext_);
			pEncodec_ = NULL;
		}
		if (pdict_)
		{
			av_dict_free(&pdict_);
			pdict_ = nullptr;
		}
		m_bInited = false;
	}

	void VideoEncoder::SetStreamCallback(StreamFrameCallback callback, void* userData)
	{
		callback_ = callback;
		userData_ = userData;
	}

	bool VideoEncoder::OnFrame(void* pData, AVFrame* pFrame)
	{
		int ret = avcodec_send_frame(pEncodeContext_, pFrame);
		if (ret < 0)
		{
			std::cerr << "EncodeStream.avcodec_send_frame fail = "<< ret << std::endl;
			return false;
		}

		while (ret >= 0) {
			AVPacket* avpkt = av_packet_alloc();
			ret = avcodec_receive_packet(pEncodeContext_, avpkt);
			if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF)
			{
				av_packet_free(&avpkt);
				return true;
			}
			else if (ret < 0)
			{
				av_packet_free(&avpkt);
				return false;
			}
			//ENCODE_STREAM_DEBUG("enstream.len=%d, pts=%" PRId64 "\n", avpkt->size, avpkt->pts);
			int codecID = 0;
			if (pEncodeContext_->codec_id == AV_CODEC_ID_H264)
				codecID = 0;
			else if (pEncodeContext_->codec_id == AV_CODEC_ID_HEVC)
				codecID = 1;

			bool isKey = false;
			if (avpkt->flags & AV_PKT_FLAG_KEY)
				isKey = true;
			if (callback_)
			{
				if(pts_ < pFrame->pts)
					pts_ = pFrame->pts;
				else
					pts_ += 1;
				callback_(0, codecID, avpkt->data, avpkt->size, pts_, isKey, userData_);
			}
			av_packet_free(&avpkt);
		}

		return true;
	}

}