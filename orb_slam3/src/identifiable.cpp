//
// Created by vahagn on 09.03.21.
//
#include <identifiable.h>
namespace orb_slam3 {

size_t Identifiable::next_id_ = 0;

Identifiable::Identifiable() : id_(++next_id_) {

}

}