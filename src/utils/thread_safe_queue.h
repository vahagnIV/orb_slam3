//
// Created by vahagn on 08.10.21.
//

#ifndef ORB_SLAM3_SRC_UTILS_THREAD_SAFE_QUEUE_H_
#define ORB_SLAM3_SRC_UTILS_THREAD_SAFE_QUEUE_H_

#include <queue>
#include <mutex>

namespace orb_slam3 {
namespace utils {
template<typename T>
class ThreadSafeQueue {
 public:
  ThreadSafeQueue() = default;
  ~ThreadSafeQueue() = default;
 public:
  typedef std::queue<T> QueueType;
  void Push(T &elem) {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    queue_.push(elem);
  }
  T &Front() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    return queue_.front();
  }

  void Pop() {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    queue_.pop();
  }

  bool Empty() const {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    return queue_.empty();
  }

  typename QueueType::size_type Size() const {
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    return queue_.size();
  }

  void Clear(){
    std::unique_lock<decltype(mutex_)> lock(mutex_);
    while (!queue_.empty())
      queue_.pop();
  }
 private:
  QueueType queue_;
  mutable std::mutex mutex_;

};
}
}

#endif //ORB_SLAM3_SRC_UTILS_THREAD_SAFE_QUEUE_H_
