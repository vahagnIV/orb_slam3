//
// Created by vahagn on 02/02/21.
//

#ifndef ORB_SLAM3_WINDOW_MATCHER_H
#define ORB_SLAM3_WINDOW_MATCHER_H

// === stl ===
#include <unordered_set>

// == orb-slam3 ===
#include <features/imatcher.h>
#include <features/iterators/idescriptor_iterator.h>

namespace orb_slam3 {
namespace features {

class SNNMatcher : public IMatcher {
 public:
  SNNMatcher(const precision_t nearest_neighbour_ratio,
             const bool check_orientation);

  template<typename IteratorType>
  void MatchWithIterator(const DescriptorSet &descriptors_to,
                                 const DescriptorSet &descriptors_from,
                                 std::vector<features::Match> &out_matches,
                                 IJointDescriptorIterator<IteratorType> *iterator);
  template<typename IteratorType>
  size_t MatchWithIteratorInternal(const DescriptorSet &descriptors_to,
                                 const DescriptorSet &descriptors_from,
                                 std::vector<int> &out_matches,
                                 IJointDescriptorIterator<IteratorType> *iterator);

  /*!
 * Matches descriptors using Lowe's second nearest neighbour algorithm
 * @param features_to The first set of features
 * @param features_from The second set of features
 * @param out_matches The output vector of matches
 */
  void Match(const features::Features &features_to,
             const features::Features &features_from,
             std::vector<features::Match> &out_matches,
             size_t window_size = 0) const override;

  /*!
 * Matches using Bag of Words.
 * @param features_from Features container for the first images.
 * @param features_to Features container for the second images.
 * @param mask_from Mask for the first container. The feature should be matched if its id exists in mask_from.
 * @param mask_to Negative mask for the second container. The feature should NOT be matched if its id is in mask_to.
 * @param out_matches The output vector of matches.
 */
  void MatchByBoW(const Features &features_from,
                  const Features &features_to,
                  const std::unordered_set<size_t> &mask_from,
                  const std::unordered_set<size_t> &mask_to,
                  std::vector<features::Match> &out_matches) const;

  /*!
   * Sets the corresponding values in inout_matches_12 to -1 if the corresponding keypoints do not pass orientation constraint
   * @param inout_matches_12 The initial matches vector
   * @param features1 First set of features
   * @param features2 Second set of features
   * @return The number of discarded matches
   */
  static int FilterByOrientation(std::vector<int> &inout_matches_12,
                                 const Features &features1,
                                 const Features &features2);

 private:

  static void ComputeThreeMaxima(std::vector<int> *histo, int &ind1, int &ind2, int &ind3);

  int SNNMatch(const features::Features &features1,
               const features::Features &features2,
               std::vector<int> &out_matches_12,
               size_t window_size = 0) const;

  void FindDescriptorInSet(const DescriptorType &d1,
                           const features::DescriptorSet &descriptors2,
                           const std::vector<size_t> &allowed_inidces,
                           int &out_idx2,
                           unsigned &dist) const;

  static inline void ComputeRotationHistogram(std::vector<int> *rotation_histogram,
                                              const std::vector<int> &inout_matches_12,
                                              const Features &features1,
                                              const Features &features2);
 public:
  static const unsigned TH_LOW;
  static const int TH_HIGH;
  static const int HISTO_LENGTH;

 private:
  const precision_t nearest_neighbour_ratio_;
  const bool check_orientation_;

};

}
}

#endif //ORB_SLAM3_WINDOW_MATCHER_H
