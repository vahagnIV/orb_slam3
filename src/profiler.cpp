//
// Created by karen on 20/10/20.
//

/// Standard includes

#include <cassert>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

/// Idem includes

#include "profiler.h"

namespace orb_slam3 {

Profiler::IdToMarks Profiler::id_to_marks_;
Profiler::IdToProfile Profiler::id_to_profile_;
std::map<std::string, unsigned int> Profiler::time_stamps_;
std::mutex Profiler::mutex_;

void Profiler::Mark(const std::string & id, const std::string & name) {

  const std::string tid = GenerateThreadId(id);
  namespace C = std::chrono;
  C::high_resolution_clock::time_point t = C::high_resolution_clock::now();
  const unsigned int utime = C::duration_cast<C::microseconds>(t.time_since_epoch()).count();

  std::lock_guard<std::mutex> lock(mutex_);
  IdToMarks::iterator it = id_to_marks_.find(tid);
  if (it != id_to_marks_.end()) {
    assert(0 != it->second.size());
    it->second.push_back(std::make_pair(name, utime));
  } else {
    MarkTypes m(1, std::make_pair(name, utime));
    id_to_marks_.insert(std::make_pair(tid, m));
  }
}

void Profiler::PrintMarks(const std::string & id) {

  const std::string tid = GenerateThreadId(id);
  std::lock_guard<std::mutex> lock(mutex_);
  IdToMarks::iterator it = id_to_marks_.find(tid);
  if (it != id_to_marks_.end()) {
    unsigned int l = 0;
    for (MarkType m: it->second) {
      if (0 != l) {
        const std::string d = std::to_string((double) (m.second - l) / 1000000);
        std::cout << "time stamp: " << tid << " " << m.first << " " << d << " sec (" << m.second << ")" << std::endl;
      } else {
        std::cout << "time stamp: " << tid << " " << m.first << " (" << m.second << ")" << std::endl;
      }
      l = m.second;
    }
  }
}

void Profiler::Start(const std::string & id) {

  const std::string tid = GenerateThreadId(id);
  namespace C = std::chrono;
  C::high_resolution_clock::time_point t = C::high_resolution_clock::now();
  const unsigned int utime = C::duration_cast<C::microseconds>(t.time_since_epoch()).count();

  std::lock_guard<std::mutex> lock(mutex_);
#ifdef DEBUG
  std::map<std::string, unsigned int>::const_iterator f = time_stamps_.find(tid);
  assert(f == time_stamps_.end() || 0 == f->second); /// TODO
#endif
  time_stamps_[tid] = utime;
  IdToProfile::iterator it = id_to_profile_.find(tid);
  if (it == id_to_profile_.end()) {
    id_to_profile_.insert(std::make_pair(tid, std::make_pair(0, 0)));
  }
}

void Profiler::End(const std::string & id) {

  const std::string tid = GenerateThreadId(id);
  namespace C = std::chrono;
  C::high_resolution_clock::time_point t = C::high_resolution_clock::now();
  const unsigned int utime = C::duration_cast<C::microseconds>(t.time_since_epoch()).count();

  std::lock_guard<std::mutex> lock(mutex_);
  std::map<std::string, unsigned int>::iterator it = time_stamps_.find(tid);
  //assert(it != time_stamps_.end());
  if (it != time_stamps_.end() && 0 != it->second) {
    const unsigned int d = utime - it->second;
    it->second = 0;
    IdToProfile::iterator f = id_to_profile_.find(tid);
    assert(f != id_to_profile_.end());
    f->second.first += d;
    ++f->second.second;
  }
}

void Profiler::PrintProfiles() {

  std::lock_guard<std::mutex> lock(mutex_);

  for (auto & it: id_to_profile_) {
    const std::string d = std::to_string((double) it.second.first / 1000000);
    std::cout << it.first << ": " << " " << d << " sec " << "(" << it.second.second << " times)" << std::endl;
    it.second.first = 0;
    it.second.second = 0;
  }
}

std::string Profiler::GenerateThreadId(const std::string & id) {

  std::stringstream x;
  x << std::this_thread::get_id();
  return x.str() + " " + id;
}

}
