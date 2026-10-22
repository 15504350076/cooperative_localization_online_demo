/*================================================================================

Module Name: SysShmWrapper.h

General Description: A simple C++ wrapper for System V shared memory.
                     Only support one reader process and one writer process now.
                     The writer process and reader process need use the same path and
                     size to initialize the shared memory.
                     The shared memory and semapore resources will not be released.
                     They can be re-used after writer or reader restarts.

==================================================================================*/

#pragma once

#include <cstdint>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

namespace ShmWrapper {

const size_t RAW_DATA_SIZE = 640*640*3+36 + 16+12; //4096 * 80

// Packed T_DjiPerceptionImageInfo
struct [[gnu::packed]] ImageInfo
{
  uint32_t index;
  uint32_t direction;
  uint32_t width; //pic width
  uint32_t height; //pic height
  uint32_t dataId;
  uint32_t sequence;
  uint32_t dataType;
  uint64_t timeStamp; //T_DjiPerceptionImageInfo end
  uint32_t videoWidth;//video width
  uint32_t videoHeight;//video height
  uint32_t dataLength; //raw data length
};

enum InitState
{
  INIT_INITIALIZED = 0,
  INIT_NOT_INITIALIZED,
  INIT_FAILED_KEY_PATH,
  INIT_FAILED_KEY_FTOK,
  INIT_FAILED_SHM_GET,
  INIT_FAILED_SHM_AT,
  INIT_FAILED_SHM_SIZE,
  INIT_FAILED_SEM_GET,
  INIT_FAILED_SEM_CTL
};

enum OperationResult
{
  OP_SUCCESS = 0,
  OP_DATA_FINISHED,           // Indicate no data any more
  OP_TIMEOUT,
  OP_FAILED_NOT_INITIALIZED,
  OP_FAILED_SEM,
  OP_FAILED_DATA,             // Wrong input data buffer
  OP_FAILED_NOT_ENOUGH_MEM
};

class ShmBase
{
public:
  ShmBase(const std::string &path, size_t shm_size, bool readonly = false);
  virtual ~ShmBase();

  // A header is added before each data frame so the max size of one data frame is
  // shared memory size - HEADER_SIZE
  static size_t GetHeaderSize() { return HEADER_SIZE; }
  size_t GetAvaliableSize() const { return shm_size_ - HEADER_SIZE; }

  InitState GetState() const { return init_state_; }
  std::string GetKeyPath() const { return key_path_; }
  key_t GetKey() const { return key_; }
  int GetShmId() const { return shm_id_; }
  int GetSemId() const { return sem_id_; }

protected:
  struct [[gnu::packed]] Header
  {
    uint64_t id;
    size_t data_size;
  };
  static const size_t HEADER_SIZE = 16;
  static_assert(sizeof(Header) == HEADER_SIZE, "");

  // No copyable
  ShmBase(const ShmBase&) = delete;                                                        \
  ShmBase& operator=(const ShmBase&) = delete;

  bool CreateKey();
  bool CreateAndAttachShm(bool readonly);
  bool CreateSem();

  void DetachShm();

  OperationResult P(unsigned short int index, const time_t *timeout_ms);
  OperationResult V(unsigned short int index);

  InitState init_state_ = INIT_NOT_INITIALIZED;
  const std::string key_path_;
  key_t key_ = -1;
  size_t shm_size_ = 0;
  int shm_id_ = -1;
  void* shm_address_ = nullptr;
  int sem_id_ = -1;
};

class ShmReader : public ShmBase
{
public:
  ShmReader(const std::string &path, size_t shm_size);

  // Read one data frame to buffer.
  // Return OP_FAILED_NOT_ENOUGH_MEM if the buffer_size is smaller than the data_size,
  // and the data is still kept in the shared memory.
  // Read will block if no data until new data or timeout if timeout_ms is set.
  OperationResult Read(void* buffer,
                       size_t buffer_size,
                       uint64_t &data_id,
                       size_t &data_size,
                       const time_t *timeout_ms = nullptr);
};

class ShmWriter : public ShmBase
{
public:
  ShmWriter(const std::string &path, size_t shm_size);

  // If previous data is existed, Write() will block until reader finishes the
  // data reading or timeout if timeout_ms is set.
  OperationResult Write(const void *data,
                        uint64_t id,
                        size_t data_size,
                        const time_t *timeout_ms = nullptr);

  // Send an empty data to indicate all data is finished, then
  // ShmReader::Read() will return OP_DATA_FINISHED.
  OperationResult Stop() { return Write(nullptr, 0, 0); }
};

class ShmWriterNoBlockObserver
{
public:
  virtual ~ShmWriterNoBlockObserver() = default;
  virtual void OnDataDiscarded(const void* data, uint64_t id, size_t size) = 0;
  virtual void OnDataFailed(const void* data, uint64_t id,
                            size_t data_size, OperationResult ret) = 0;
};

// ShmWriter::Write() will block or timeout if the reader can't read the data
// on timne. ShmWriterNoBlock will create a writer thread to write the data,
// then the main thread will not be blocked. The write thread keeps at most
// latest max_cache_frame_num data frames on hand if can't write them on time,
// so some old data will be discarded.
class ShmWriterNoBlock
{
public:
  ShmWriterNoBlock(const std::string &path,
                   size_t shm_size,
                   size_t max_cache_frame_num,
                   time_t timeout_ms,
                   ShmWriterNoBlockObserver* observer = nullptr);

  const ShmWriter& GetWriter() const { return writer_; }

  // Return false is the input data is wrong or Stop() has been called already
  bool Write(const void *data, size_t data_size);

  // "discard_data == true" means discarding all remaining data,
  // otherwise stop after sending them.
  void Stop(bool discard_data = false);

private:
  enum State {RUNNING, STOP_AFTER_FINISH_DATA, STOP_NOW};
  struct DataFrame
  {
    DataFrame(const uint8_t *data, uint64_t data_id, size_t data_size);
    const uint64_t id;
    std::vector<uint8_t> data_buffer; // TODO: buffer pool
  };

  void Push(std::shared_ptr<DataFrame> data_p);
  void PopUntil(size_t keep_size);

  void Run();

  ShmWriter writer_;
  const size_t max_cache_frame_num_;
  const time_t timeout_ms_;
  ShmWriterNoBlockObserver* observer_;
  uint64_t id_ = 0;
  State state_ = RUNNING;
  std::queue<std::shared_ptr<DataFrame>> data_frames_;

  std::recursive_mutex mutex_;
  std::thread write_thread_;
};

} // namespace ShmWrapper
