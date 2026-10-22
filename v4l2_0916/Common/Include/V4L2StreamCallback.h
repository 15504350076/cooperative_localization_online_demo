#pragma once
#include <stdint.h>
#include "ffmpeg.h"
typedef void (*V4L2FrameCallback)(int arrayIndex, AVFrame* frame, uint64_t capture_timestamp, void* userData);
