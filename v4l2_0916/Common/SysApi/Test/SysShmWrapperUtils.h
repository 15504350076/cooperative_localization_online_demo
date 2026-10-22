/*
Case 1：读写阻塞，有些老数据被丢掉，但 main thread （比如来自大疆）不会被阻塞
   Way 1. 先运行 SysShmWrapperWriterNoBlock 再运行 SysShmWrapperReader。
   Way 2. 先运行 SysShmWrapperReader 1, 再运行 SysShmWrapperWriterNoBlock。

Case 2: 读写正常
  先运行 SysShmWrapperReader 再运行 SysShmWrapperWriterNoBlock。

参数都可调整， 在 SysShmWrapperUtils.h 里。

已知的问题：
. 有个 TODO 去提高 cache buffer 性能。
*/

#pragma once

#include <string>

#define KEY_PATH "/run/shmwrapper_test"
#define SHM_SIZE (1024 * 1024 * 5)

#define TOTLA_FRAME_NUM 20000
#define MAX_CACHE_FRAME_NUM 1
#define FRAME_PER_SECOND 50

#define TIMEOUT_MS 50
#define DELAY_MS (TIMEOUT_MS + TIMEOUT_MS / 2)

// Utils for test

namespace ShmWrapper {

// Generate a buffer. The size is random and enough big to match (<=) the
// max_size, also add a random char in the buffer beginning and end, so
// GetBufferMarkedChars() can return a string like 'a..f' to identify the buffer
size_t MarkBuffer(char* buffer, size_t max_size);

std::string GetBufferMarkedChars(const char* buffer, size_t size);

} // namespace ShmWrapper
