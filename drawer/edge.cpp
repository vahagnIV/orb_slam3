//
// Created by vahagn on 16/08/2021.
//

#include "edge.h"

namespace orb_slam3 {
namespace drawer {

Edge::Edge(const Node * node1, const Node * node2) : node1_(node1), node2_(node2) {

}

const Node * Edge::Node1() const {
  return node1_;
}

const Node * Edge::Node2() const {
  return node2_;
}

}
}