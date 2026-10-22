#pragma once

extern "C"
{

#pragma warning( push )
#pragma warning( disable : 4244 )
#include <stdint.h>
#include "libavcodec/avcodec.h"
#include "libavcodec/bsf.h"
#include "libavutil/avutil.h"
#include "libswscale/swscale.h"
#include "libavutil/cpu.h"
	//#include "libavcodec/dxva2.h" //will include d3d9.h, dxva2api.h
#include "libavutil/pixdesc.h"

#include "libswresample/swresample.h"
#include "libavformat/avformat.h"

#include "libavutil/opt.h"
#include "libavutil/fifo.h"
#include "libavutil/avstring.h"
#include "libavutil/dict.h"

#include "libavfilter/avfilter.h"
#include "libavfilter/buffersink.h"
#include "libavfilter/buffersrc.h"

#pragma warning( pop )

}