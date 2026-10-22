#pragma once
#include <stdint.h>
#include <cstddef>

enum EMDataFormat :unsigned char
{
	enumDataFormatUnknown = 0,
	enumDataFormatVideoBegin = 1,
	enumDataFormatVideoH264 = 1,
	enumDataFormatVideoH265 = 2,
	enumDataFormatVideoMjpeg = 3,
	enumDataFormatVideoEnd = 99,

	enumDataFormatAudioBegin = 100,
	enumDataFormatAudioG711U = 100,
	enumDataFormatAudioG711A = 101,
	enumDataFormatAudioG726 = 102,
	enumDataFormatAudioG722 = 103,
	enumDataFormatAudioAAC = 104,
	enumDataFormatAudioPCM = 105,
	enumDataFormatAudioEnd = 150,

	enumDataFormatMetadataBegin = 200,
	enumDataFormatMetadataCommand = 200,
	enumDataFormatMetadataEnd = 254,

	enumDataFormatGlobalHeader = 0xff


};

/*
datatype��int64_t

	heigh                                                                                                          low
			 width(2bytes)                    height(2bytes)      reserved(1)    reserved(1)       I-P            codec
	| ------------ | ------------ | ------------ | ------------ | ------------ | ------------ | ------------ | ------------ |
	0              1              2              3              4              5              6              7              8
*/

//const unsigned char FRAME_TYPE_I_FRAME = 1;
//const unsigned char FRAME_TYPE_P_FRAME = 0;
//const unsigned char FRAME_TYPE_GLOBAL_HEADER = 0x000000FF;
//const unsigned char FRAME_TYPE_UNKNOWN = 0;


class MediaPacket {
public:
	MediaPacket()
		: dataType(0)
		, pts(0)
		, data(nullptr)
		, size(0)
		, extradata(nullptr)
		, extradata_size(0)
	{
	}
	uint64_t dataType;
	uint64_t pts; //ms

	unsigned char* data;
	size_t size;

	unsigned char* extradata; //not used currently
	size_t extradata_size;
};

struct MediaSample {
	unsigned char* data;
	size_t size;
	bool isVideo;
	bool isKey;
	uint64_t pts;

	unsigned char* extradata; //not used currently
	size_t extradata_size;
	EMDataFormat codecID; //int codecID;
};

typedef void (*StreamFrameCallback)(int arrayIndex, int codecID, const uint8_t* buf, int bufLen,int64_t ps, bool isKey, void* userData);
typedef void* HANDLE;
