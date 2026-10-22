#include "SysApi/SysShmWrapper.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <sys/sem.h>
#include <sys/shm.h>

namespace ShmWrapper {

#define SYS_V_SEM_MOD 0644

const int SEM_NUM = 2;
const unsigned short int SEM_WRITER = 0;  // The first is for writer to write.
const unsigned short int SEM_READER = 1;  // The second is for reader to read.

union semun
{
  int val;
  struct semid_ds *buf;
  unsigned short *arry;
  struct seminfo *__buf;
};

ShmBase::ShmBase(const std::string &path, size_t shm_size, bool readonly)
  : key_path_(path),
    shm_size_(shm_size)
{
  if (CreateKey()
      && CreateAndAttachShm(readonly)
      && CreateSem()) {
    init_state_ = INIT_INITIALIZED;

    // If previous writer or reader exits unnormally, the semapore value maybe
    // is wrong then will block both writer and reader. Try to restore it.
    // Please note at current time, maybe another is in the reading or writing
    // now, so check the value at most three times. Normally it is enough.
    int writer_value = 0;
    int reader_value = 0;
    for (int i = 0; i < 3; ++i) {
      writer_value = semctl(sem_id_, SEM_WRITER, GETVAL, 0);
      reader_value = semctl(sem_id_, SEM_READER, GETVAL, 0);
       if (writer_value != 0 || reader_value != 0) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    if (writer_value == 0 && reader_value == 0) {
      std::cout << "Rstrore writer to continue" << std::endl;
      V(SEM_WRITER);
    }
  }
}

ShmBase::~ShmBase()
{
  DetachShm();
}

bool ShmBase::CreateKey()
{
  // It is fine that the existed key_path_ has read only permission
  std::ifstream fin;
  fin.open(key_path_, std::ios::in);
  if (!fin.is_open()) {
    // Create if not existed
    std::ofstream fout(key_path_);
    fout.close();
  }
  fin.close();

  key_ = ftok(key_path_.c_str(), 0);
  if (key_ == -1) {
    init_state_ = INIT_FAILED_KEY_FTOK;
    return false;
  }

  return true;
}

bool ShmBase::CreateAndAttachShm(bool readonly)
{
  shm_id_ = shmget(key_, shm_size_, IPC_CREAT | 0666);
  if (shm_id_ == -1) {
    init_state_ = INIT_FAILED_SHM_GET;
    return false;
  }

  shm_address_ = shmat(shm_id_, nullptr, readonly ? SHM_RDONLY : 0);
  if (shm_address_ == reinterpret_cast<void*>(-1)) {
    shm_address_ = nullptr;
    init_state_ = INIT_FAILED_SHM_AT;
    return false;
  }

  // Update to the real size
  shmid_ds ds;
  if (shmctl(shm_id_, IPC_STAT, &ds) == -1) {
    DetachShm();
    init_state_ = INIT_FAILED_SHM_SIZE;
    return false;
  }
  shm_size_ = ds.shm_segsz;

  std::ofstream fout(key_path_ + ".shmid");
  fout << shm_id_;
  fout.close();
  return true;
}

bool ShmBase::CreateSem()
{
  sem_id_ = semget(key_, SEM_NUM, IPC_CREAT | IPC_EXCL | SYS_V_SEM_MOD);
  if (sem_id_ != -1) {
    union semun semopts_w;
    semopts_w.val = 1; // can write
    union semun semopts_r;
    semopts_r.val = 0; // no data to read
    if (semctl(sem_id_, SEM_WRITER, SETVAL, semopts_w) == -1
        || semctl(sem_id_, SEM_READER, SETVAL, semopts_r) == -1) {
      DetachShm();
      init_state_ = INIT_FAILED_SEM_CTL;
      return false;
    }
  }
  else if (errno == EEXIST) {
    sem_id_ = semget(key_, SEM_NUM, SYS_V_SEM_MOD);
  }

  if (sem_id_ == -1) {
    DetachShm();
    init_state_ = INIT_FAILED_SEM_GET;
    return false;
  }

  std::ofstream fout(key_path_ + ".semid");
  fout << sem_id_;
  fout.close();
  return true;
}

void ShmBase::DetachShm()
{
  if (shm_address_) {
    shmdt(shm_address_);
    shm_address_ = nullptr;
  }
}

OperationResult ShmBase::P(unsigned short int index, const time_t *timeout_ms)
{
  struct sembuf sem_b = {index, -1, 0};
  struct timespec timeout = {
    timeout_ms ? *timeout_ms / 1000 : 0,            // second
    timeout_ms ? (*timeout_ms % 1000) * 1000000 : 0 // nanosecond
  };
  const auto ret = semtimedop(sem_id_, &sem_b, 1,
                              timeout_ms ? &timeout : nullptr);
  if (ret != -1) {
    return OP_SUCCESS;
  }
  else if (errno == EAGAIN) {
    return OP_TIMEOUT;
  }
  else {
    return OP_FAILED_SEM;
  }
}

OperationResult ShmBase::V(unsigned short int index)
{
  struct sembuf sem_b = {index, 1, 0};
  return semop(sem_id_, &sem_b, 1) != -1 ? OP_SUCCESS : OP_FAILED_SEM;
}

ShmReader::ShmReader(const std::string &path, size_t shm_size)
  : ShmBase(path, shm_size, true)
{
}

OperationResult ShmReader::Read(void* buffer,
                                size_t buffer_size,
                                uint64_t &data_id,
                                size_t &data_size,
                                const time_t *timeout_ms)
{
  if (init_state_ != INIT_INITIALIZED) {
    return OP_FAILED_NOT_INITIALIZED;
  }

  if (!buffer) {
    return OP_FAILED_DATA;
  }

  auto ret = P(SEM_READER, timeout_ms);
  if (ret != OP_SUCCESS) {
    return ret;
  }

  Header header;
  memcpy(&header, shm_address_, HEADER_SIZE);
  data_id = header.id;
  data_size = header.data_size;
  if (header.data_size == 0) {
    V(SEM_WRITER);
    return OP_DATA_FINISHED;
  }

  if (header.data_size > buffer_size) {
    ret = V(SEM_READER); // Then can read again
    return ret == OP_SUCCESS ? OP_FAILED_NOT_ENOUGH_MEM : ret;
  }

  memcpy(buffer, static_cast<uint8_t*>(shm_address_) + HEADER_SIZE, data_size);
  return V(SEM_WRITER);
}

ShmWriter::ShmWriter(const std::string &path, size_t shm_size)
  : ShmBase(path, shm_size)
{
}

OperationResult ShmWriter::Write(const void *data,
                                 uint64_t id,
                                 size_t data_size,
                                 const time_t *timeout_ms)
{
  if (init_state_ != INIT_INITIALIZED) {
    return OP_FAILED_NOT_INITIALIZED;
  }

  if (!data && data_size != 0) {
     return OP_FAILED_DATA;
  }

  if (GetAvaliableSize() < data_size) {
    return OP_FAILED_NOT_ENOUGH_MEM;
  }

  const auto ret = P(SEM_WRITER, timeout_ms);
  if (ret != OP_SUCCESS) {
    return ret;
  }

  Header header = {id, data_size};
  memcpy(shm_address_, &header, HEADER_SIZE);
  if (data_size != 0) {
    memcpy(static_cast<uint8_t*>(shm_address_) + HEADER_SIZE, data, data_size);
  }

  return V(SEM_READER);
}

ShmWriterNoBlock::ShmWriterNoBlock(const std::string &path,
                                   size_t shm_size,
                                   size_t max_cache_frame_num,
                                   time_t timeout_ms,
                                   ShmWriterNoBlockObserver* observer)
  : writer_(path, shm_size),
    max_cache_frame_num_(max_cache_frame_num),
    timeout_ms_(timeout_ms),
    observer_(observer)
{
  write_thread_ = std::thread(&ShmWriterNoBlock::Run, this);
}

bool ShmWriterNoBlock::Write(const void *data, size_t data_size)
{
  if (!data || data_size == 0 || state_ != RUNNING) {
    return false;
  }

  auto data_p = std::make_shared<DataFrame>(static_cast<const uint8_t*>(data),
                                            ++id_, data_size);

  std::lock_guard<std::recursive_mutex> guard(mutex_);
  PopUntil(max_cache_frame_num_ - 1);
  Push(data_p);
  return true;
}

void ShmWriterNoBlock::Stop(bool discarded_data)
{
  {
    std::lock_guard<std::recursive_mutex> guard(mutex_);
    if (discarded_data) {
      PopUntil(0);
      state_ = STOP_NOW;
    } else {
      // An empty data frame to notify reader that no data any more.
      // Same as ShmWriter::Stop()
      auto data_p = std::make_shared<DataFrame>(nullptr, ++id_, 0);
      Push(data_p);
      state_= STOP_AFTER_FINISH_DATA;
    }
  }
  write_thread_.join();
}

void ShmWriterNoBlock::Push(std::shared_ptr<DataFrame> data_p)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  data_frames_.emplace(data_p);
}

void ShmWriterNoBlock::PopUntil(size_t keep_size)
{
  std::lock_guard<std::recursive_mutex> guard(mutex_);
  while (data_frames_.size() > keep_size) {
    if (observer_) {
      observer_->OnDataDiscarded(&data_frames_.front()->data_buffer[0],
                                 data_frames_.front()->id,
                                 data_frames_.front()->data_buffer.size());
    }
    data_frames_.pop();
  }
}

void ShmWriterNoBlock::Run()
{
  std::shared_ptr<DataFrame> current_frame;
  while (true) {
    {
      std::lock_guard<std::recursive_mutex> guard(mutex_);
      if (state_ == STOP_NOW) {
        if (current_frame && observer_) {
          observer_->OnDataDiscarded(&current_frame->data_buffer[0],
                                     current_frame->id,
                                     current_frame->data_buffer.size());
        }
        current_frame.reset();
        break;
      }
      // All data are sent
      else if (state_ == STOP_AFTER_FINISH_DATA && data_frames_.empty() && !current_frame) {
        break;
      }

      // If next frame was discarded already, then delete current frame which is
      // not sent
      if (!data_frames_.empty() && current_frame
          && data_frames_.front()->id > current_frame->id + 1) {
        if (observer_) {
          observer_->OnDataDiscarded(&current_frame->data_buffer[0],
                                     current_frame->id,
                                     current_frame->data_buffer.size());
        }
        current_frame.reset();
      }

      if (!current_frame && !data_frames_.empty()) {
        current_frame = data_frames_.front();
        data_frames_.pop();
      }
    }

    if (!current_frame) {
      // No data currently. Sleep a while to avoid high cpu loading
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      continue;
    }

    const auto ret = writer_.Write(&current_frame->data_buffer[0],
                                   current_frame->id,
                                   current_frame->data_buffer.size(),
                                   &timeout_ms_);
    if (ret == ShmWrapper::OP_TIMEOUT) {
      continue;
    }

    if (ret != ShmWrapper::OP_SUCCESS && observer_) {
      observer_->OnDataFailed(&current_frame->data_buffer[0],
                              current_frame->id,
                              current_frame->data_buffer.size(),
                              ret);
      break;
    }

    current_frame.reset();
  }
}

ShmWriterNoBlock::DataFrame::DataFrame(const uint8_t *data,
                                       uint64_t data_id,
                                       size_t data_size)
  : id(data_id),
    data_buffer(data_size) // better performance to init a big vector in constructor
{
  if (data && data_size > 0) {
    memcpy(&data_buffer[0], data, data_size);
  }
}

} // namespace ShmWrapper
