//
// Created by karen on 20/10/20.
//

#ifndef NVISION_PROFILER_H
#define NVISION_PROFILER_H

/// Standard includes

#include <vector>
#include <map>
#include <mutex>

namespace orb_slam3 {

/**
 * Profiler class allows you to calculate the durations of functions or any part of the code.
 * It is thread safe
 *
 * There is support for 2 usecases:
 *
 *      - Calculation with Marks
 *
 *              Profiler::Mark("profile");
 *              f1();
 *              Profiler::Mark("profile", "f1");
 *              f2();
 *              Profiler::Mark("profile", "f2");
 *              f3();
 *              Profiler::Mark("profile", "f3");
 *              Profiler::PrintMarks("profile");
 *
 *
 *      - Calculation with Start/Stop mechanism
 *
 *              while() {
 *                      Profiler::Start("f1");
 *                      f1();
 *                      Profiler::End("f1");
 *                      f2();
 *                      Profiler::Start("f3/4");
 *                      f3();
 *                      Profiler::Start("f4/5");
 *                      f4();
 *                      Profiler::End("f3/4");
 *                      f5();
 *                      Profiler::End("f4/5");
 *              }
 *              Profiler::PrintProfiles();
 */

class Profiler {

 /// Profiling with marks
 public:
  typedef std::pair<std::string, unsigned int> MarkType;
  typedef std::vector<MarkType> MarkTypes;
  typedef std::map<std::string, MarkTypes> IdToMarks;

  static void Mark(const std::string & id, const std::string & name = "");
  static void PrintMarks(const std::string & id);

 private:
  static IdToMarks id_to_marks_;


  /// Profiling with start/end mechanism
 public:
  typedef std::pair<unsigned int, unsigned int> Profile;
  typedef std::map<std::string, Profile> IdToProfile;

  static void Start(const std::string & id = "");
  static void End(const std::string & id = "");
  static void PrintProfiles();

 private:
  static IdToProfile id_to_profile_;
  static std::map<std::string, unsigned int> time_stamps_;

  /// Helper static members
 private:
  static std::string GenerateThreadId(const std::string &);

 private:
  static std::mutex mutex_;
};
}

#endif //NVISION_PROFILER_H
