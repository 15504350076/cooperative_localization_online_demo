#include "SysShmWrapperUtils.h"

#include <cstring>
#include <ctime>
#include <random>

namespace ShmWrapper {

size_t MarkBuffer(char* buffer, size_t max_size)
{
  const auto max_char_num = 'z' - 'a';

  if (!buffer) {
    return 0;
  }

  if (max_size < max_char_num) {
    memset(buffer, '.', max_size);
    return max_size;
  }

  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> dis(0, max_char_num);

  const auto ret = dis(gen);
  const size_t gen_size = max_size - max_char_num + ret;

  buffer[0] = 'a' + ret;
  buffer[gen_size - 1] = 'a' + dis(gen);

  return gen_size;
}

std::string GetBufferMarkedChars(const char* buffer, size_t size)
{
  std::string ret("....");
  if (buffer && size > 3) {
    ret[0] = buffer[0];
    ret[3] = buffer[size -1];
  }
  return ret;
}

} // namespace ShmWrapper
