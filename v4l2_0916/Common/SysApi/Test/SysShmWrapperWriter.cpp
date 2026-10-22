#include "SysApi/SysShmWrapper.h"
#include "SysShmWrapperUtils.h"

#include <iostream>

int main()
{
  ShmWrapper::ShmWriter writer(KEY_PATH, SHM_SIZE);
  if (writer.GetState() != ShmWrapper::INIT_INITIALIZED) {
    std::cerr << "Writer failed: " << writer.GetState() << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Writer: key_path=" << writer.GetKeyPath()
            << " key=" << writer.GetKey()
            << " shm_id=" << writer.GetShmId()
            << " shm_avaliable_size=" << writer.GetAvaliableSize()
            << " sem_id=" << writer.GetSemId() << std::endl;

  const time_t timeout_ms = TIMEOUT_MS;
  const size_t buffer_size = SHM_SIZE - ShmWrapper::ShmBase::GetHeaderSize();
  char buffer[buffer_size];
  uint64_t id = 1;
  while (id <= TOTLA_FRAME_NUM) {
    const auto size = ShmWrapper::MarkBuffer(buffer, buffer_size);
    const auto buffer_mark = ShmWrapper::GetBufferMarkedChars(buffer, size);
    const auto ret = writer.Write(buffer, id, size, &timeout_ms);
    switch (ret) {
      case ShmWrapper::OP_SUCCESS:
        std::cout << "Writer: sent    '" << buffer_mark
                  << "'(id=" << id << ", size=" << size << ")" << std::endl;
        ++id;
        break;
    case ShmWrapper::OP_TIMEOUT:
      std::cout << "Writer: timeout" << std::endl;
      break;
    default:
      std::cerr << "Writer fail: " << ret << std::endl;
      return EXIT_FAILURE;
    }
  }

  writer.Stop();
  return EXIT_SUCCESS;
}
