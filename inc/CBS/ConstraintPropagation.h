// #include <vector>

#ifndef CONS_PROP_H
#define CONS_PROP_H

#include <boost/unordered_set.hpp>
#include "MDD.h"

namespace lns
{

typedef std::pair<MDDNode*, MDDNode*> node_pair;
typedef std::pair<node_pair, node_pair> edge_pair;

}

namespace std
{

using namespace lns;

template<>
struct hash<std::pair<MDDNode*, MDDNode*>> {
    size_t operator()(std::pair<MDDNode*, MDDNode*> const& p) const noexcept {
        auto a = hash<MDDNode*>{}(p.first);
        auto b = hash<MDDNode*>{}(p.second);
        return hash_combine(a, b);
    }

    static size_t hash_combine(size_t seed, size_t v) noexcept {
        // see https://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine
        seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

template<>
struct hash<std::pair<node_pair, node_pair>> {
    size_t operator()(std::pair<node_pair, node_pair> const& p) const noexcept {
        auto a = hash<node_pair>{}(p.first);
        auto b = hash<node_pair>{}(p.second);
        return hash_combine(a, b);
    }

    static size_t hash_combine(size_t seed, size_t v) noexcept {
        // see https://www.boost.org/doc/libs/1_55_0/doc/html/hash/reference.html#boost.hash_combine
        seed ^= v + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        return seed;
    }
};

}

namespace lns
{

using namespace lns;

class ConstraintPropagation{
private:
  // TODO ownership?
  // std::vector<MDD*> mdds;
  MDD* mdd0;
  MDD* mdd1;


  // check whether two nodes could be mutexed
  // return true if they are mutexed
  bool should_be_fwd_mutexed(MDDNode*, MDDNode*);

  bool should_be_fwd_mutexed(MDDNode* node_a, MDDNode* node_a_to,
                         MDDNode* node_b, MDDNode* node_b_to);

  bool should_be_bwd_mutexed(MDDNode*, MDDNode*);
  bool should_be_bwd_mutexed(MDDNode* node_a, MDDNode* node_a_to,
                             MDDNode* node_b, MDDNode* node_b_to);


  void add_bwd_node_mutex(MDDNode* node_a, MDDNode* node_b);

  void add_fwd_node_mutex(MDDNode* node_a, MDDNode* node_b);
  void add_fwd_edge_mutex(MDDNode* node_a, MDDNode* node_a_to,
                      MDDNode* node_b, MDDNode* node_b_to);

  // unordered_set<node_pair> node_cons;

  /*
    A Mutex is in the form of <<node*, node*>, <node*, node*>>
    if second node* in each pair are nullptr, then it is a node mutex

   */

  // void fwd_mutex_prop_generalized_helper(MDD* mdd_0, MDD* mdd_1, int level);

public:
  ConstraintPropagation(MDD* mdd0, MDD* mdd1):
    mdd0(mdd0), mdd1(mdd1)
  {}

  unordered_set<edge_pair> fwd_mutexes;
  unordered_set<edge_pair> bwd_mutexes;

  void init_mutex();
  void fwd_mutex_prop();
  // void fwd_mutex_prop_generalized();

  void bwd_mutex_prop();

  bool has_mutex(edge_pair);
  bool has_mutex(MDDNode*, MDDNode*);

  bool has_fwd_mutex(edge_pair);
  bool has_fwd_mutex(MDDNode*, MDDNode*);

  // MDD 0 of level_0 and MDD 1 of level_1 mutexed at goal
  bool mutexed(int level_0, int level_1);
  bool feasible(int level_0, int level_1);
  int _feasible(int level_0, int level_1);

  bool semi_cardinal(int level, int loc);


  std::pair<std::vector<Constraint>, std::vector<Constraint>> generate_constraints(int, int);
};

}

#endif
