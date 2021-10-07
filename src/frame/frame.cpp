//
// Created by vahagn on 11/05/2021.
//
#include "frame.h"

size_t orb_slam3::frame::Frame::next_id_ = 0;

namespace orb_slam3 {
namespace frame {

Frame::Frame(std::istream &stream, serialization::SerializationContext &context) : BaseFrame(stream, context) {

}

}
}
