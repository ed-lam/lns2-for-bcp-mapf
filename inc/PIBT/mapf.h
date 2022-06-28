#pragma once
#include "lns2/inc/PIBT/problem.h"

namespace lns
{

using namespace lns;

class MAPF : public Problem {
private:
  void init();

public:
  MAPF(Graph* _G,
       PIBT_Agents _A,
       std::vector<Task*> _T);
  MAPF(Graph* _G,
       PIBT_Agents _A,
       std::vector<Task*> _T,
       std::mt19937* _MT);
  ~MAPF();
  bool isSolved();
  void update();
  bool allocated() { return true; }

  std::string logStr();
};

}
