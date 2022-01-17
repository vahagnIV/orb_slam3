//
// Created by vahagn on 08.10.21.
//

#ifndef ORB_SLAM3_SRC_UTILS_THREAD_SAFE_QUEUE_H_
#define ORB_SLAM3_SRC_UTILS_THREAD_SAFE_QUEUE_H_

#include <queue>
#include <shared_mutex>

namespace orb_slam3 {
namespace utils {

template<typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue() = default;
  virtual ~ThreadSafeQueue() = default;
 public:

  typedef std::queue<T> QueueType;

  void Push(T &elem) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    queue_.push(elem);
  }

  T &Front() {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return queue_.front();
  }

  void Pop() {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    queue_.pop();
  }

  bool Empty() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return queue_.empty();
  }

  typename QueueType::size_type Size() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return queue_.size();
  }

  void Clear(){
    std::unique_lock<std::shared_mutex> lock(mutex_);
    while (!queue_.empty())
      queue_.pop();
  }
 private:
  QueueType queue_;
  mutable std::shared_mutex mutex_;

};
}
}

#endif //ORB_SLAM3_SRC_UTILS_THREAD_SAFE_QUEUE_H_
