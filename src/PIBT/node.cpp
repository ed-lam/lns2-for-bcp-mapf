/*
 * node.cpp
 *
 * Purpose: Node
 * Created by: Keisuke Okumura <okumura.k@coord.c.titech.ac.jp>
 */


#include "lns2/inc/PIBT/node.h"

namespace lns
{

using namespace lns;

int Node::cntIndex = 0;

Node::Node(int _id) : id(_id), index(cntIndex) {
  ++cntIndex;
  pos = Vec2f(0, 0);
}

}
