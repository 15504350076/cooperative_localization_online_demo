#include "SysApi/SysShmWrapper.h"
#include "SysShmWrapperUtils.h"

#include <iostream>

int main(int argc, char* argv[])
{
  bool delay = false;
  if (argc == 2) {
    delay = atoi(argv[1]);
  }
  if (delay) {
    std::cout << "Reader: dealy a while after reading then simulate time out in writer"
              << std::endl;
  }

  ShmWrapper::ShmReader reader(KEY_PATH, SHM_SIZE);
  if (reader.GetState() != ShmWrapper::INIT_INITIALIZED) {
    std::cerr << "Reader failed: " << reader.GetState() << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Reader: key_path=" << reader.GetKeyPath()
            << " key=" << reader.GetKey()
            << " shm_id=" << reader.GetShmId()
            << " shm_avaliable_size=" << reader.GetAvaliableSize()
            << " sem_id=" << reader.GetSemId() << std::endl;

  const time_t timeout_ms = TIMEOUT_MS;
  char buffer[SHM_SIZE];
  while (true) {
    uint64_t id;
    size_t size;
    const auto ret = reader.Read(buffer, SHM_SIZE, id, size, &timeout_ms);
    switch (ret) {
      case ShmWrapper::OP_DATA_FINISHED:
        std::cout << "Reader: no data any more" << std::endl;
        return EXIT_SUCCESS;
      case ShmWrapper::OP_SUCCESS:
        std::cout << "Reader: got     '"
                  << ShmWrapper::GetBufferMarkedChars(buffer, size)
                  << "'(id=" << id << ", size=" << size << ")" << std::endl;
        if (delay) {
          std::this_thread::sleep_for(std::chrono::milliseconds(DELAY_MS));
        }
        break;
      case ShmWrapper::OP_TIMEOUT:
        std::cout << "Reader: timeout" << std::endl;
        break;
      default:
        std::cerr << "Reader fail: " << ret << std::endl;
        return EXIT_FAILURE;
    }
  }

  return EXIT_SUCCESS;
}
