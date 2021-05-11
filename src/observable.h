//
// Created by vahagn on 19/04/2021.
//

#ifndef ORB_SLAM3_ORB_SLAM3_INCLUDE_OBSERVABLE_H_
#define ORB_SLAM3_ORB_SLAM3_INCLUDE_OBSERVABLE_H_
#include "observer.h"
namespace orb_slam3 {
template<typename MessageType>
class Observable {
 public:
  /*!
 * Add an observer
 * @param observer
 */
  void AddObserver(Observer<MessageType> * observer) {
    observers_.insert(observer);
  }

  /*!
   * Remove the observer
   * @param observer
   */
  void RemoveObserver(Observer<MessageType> * observer) {
    observers_.erase(observer);
  }

  void NotifyObservers(const MessageType & message) {
    for (Observer<MessageType> * observer: observers_) {
      observer->GetUpdateQueue().enqueue(message);
    }
  }
 protected:
  std::unordered_set<Observer<MessageType> *> observers_;

};

}
#endif //ORB_SLAM3_ORB_SLAM3_INCLUDE_OBSERVABLE_H_
