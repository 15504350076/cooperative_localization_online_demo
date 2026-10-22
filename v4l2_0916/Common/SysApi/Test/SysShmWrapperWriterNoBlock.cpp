#include "SysApi/SysShmWrapper.h"
#include "SysShmWrapperUtils.h"

#include <iostream>

class ShmWriterNoBlockObserver : public ShmWrapper::ShmWriterNoBlockObserver
{
public:
  void OnDataDiscarded(const void* data, uint64_t id, size_t size) override
  {
    std::cout << "OnDataDiscarded: '"
              << ShmWrapper::GetBufferMarkedChars((const char*)data, size)
              << "'(id=" << id << ", size=" << size << ")" << std::endl;
  }
  void OnDataFailed(const void* data, uint64_t id,
                    size_t size, ShmWrapper::OperationResult ret) override
  {
    std::cout << "OnDataFailed:    '"
              << ShmWrapper::GetBufferMarkedChars((const char*)data, size)
              << "'(id=" << id << ", size=" << size
              << " result=" << ret << ")" << std::endl;
  }
};

// Simulate that main thread will not be blocked
void Main_Thread(ShmWrapper::ShmWriterNoBlock& writer) {
  const size_t buffer_size = SHM_SIZE - ShmWrapper::ShmBase::GetHeaderSize();
  char buffer[buffer_size];
  for (int i = 0; i < TOTLA_FRAME_NUM; ++i) {
    const auto size = ShmWrapper::MarkBuffer(buffer, buffer_size);
    std::cout << "Main thread:     '"
              << ShmWrapper::GetBufferMarkedChars((const char*)buffer, size)
              << "'(size=" << size << ")" << std::endl;
    writer.Write(buffer, size);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000 / FRAME_PER_SECOND));
  }
  std::cout << "Main thread: exit" << std::endl;
}

int main()
{
  ShmWriterNoBlockObserver observer;
  ShmWrapper::ShmWriterNoBlock writer(KEY_PATH, SHM_SIZE, MAX_CACHE_FRAME_NUM,
                                      TIMEOUT_MS, &observer);
  if (writer.GetWriter().GetState() != ShmWrapper::INIT_INITIALIZED) {
    std::cerr << "Writer failed: " << writer.GetWriter().GetState() << std::endl;
    return EXIT_FAILURE;
  }
  std::cout << "Writer: key_path=" << writer.GetWriter().GetKeyPath()
            << " key=" << writer.GetWriter().GetKey()
            << " shm_id=" << writer.GetWriter().GetShmId()
            << " shm_avaliable_size=" << writer.GetWriter().GetAvaliableSize()
            << " sem_id=" << writer.GetWriter().GetSemId() << std::endl;

  std::thread push_thread(Main_Thread, std::ref(writer));
  push_thread.join();

  std::cout<< "Check the cpu loading when no data any more" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(60000));

  writer.Stop();
  // writer.Stop(true);  // Will discard all data if not run SysShmWrapperReader
  return EXIT_SUCCESS;
}
