#pragma once

#include "lns2/inc/PIBT/vec2f.h"
#include <iostream>
#include <vector>

namespace lns
{

using namespace lns;

class Node {
private:
  const int id;
  const int index;
  std::vector<Node*> neighbor;
  Vec2f pos;

  static int cntIndex;

public:
  Node();
  Node(int _id);
  ~Node() {};

  std::vector<Node*> getNeighbor() { return neighbor; }
  void setNeighbor(std::vector<Node*> nodes) { neighbor = nodes; }

  int getId() { return id; }
  int getIndex() { return index; }

  Vec2f getPos() { return pos; }
  void  setPos(Vec2f _pos) { pos = _pos; }
  void  setPos(int x, int y) { pos = Vec2f(x, y); }

  bool operator==(Node* v) const { return v->getId() == id; };
  bool operator!=(Node* v) const { return v->getId() != id; };
  bool operator==(Node& v) const { return v.getId() == id; };
  bool operator!=(Node& v) const { return v.getId() != id; };
};

}
