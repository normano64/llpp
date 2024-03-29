#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include <map>

#include "ped_tree.h"
#include "ped_agent.h"


namespace Ped{
  class Tagent;
  class Ttree;

  enum IMPLEMENTATION {OPENCL, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Tagent*> agentsInScenario, IMPLEMENTATION imp);
    void tick();
    const std::vector<Tagent*> getAgents() const;

    // Updates the treehash, which maps each agent to the current tree node that contains it
    void setResponsibleTree(Ped::Ttree *tree, const Ped::Tagent *agent);

    // Adds an agent to the tree structure
    void placeAgent(const Ped::Tagent *a);

    // Cleans up the tree and restructures it. Worth calling every now and then.
    void cleanup();
    ~Model();

    void setNumThreads(size_t threads);
    size_t length;
	  void enableRebuild(){Ped::Model::rebuild = true;}
	  void disableRebuild(){Ped::Model::rebuild = false;}
	  void enablePrune() {Ped::Model::prune = true;}
	  void disablePrune() { Ped::Model::prune = false;}
	  void enableTest(){Ped::Model::test = true;}
	  void setAgentsPerRegion(int num){Ped::Model::agentsPerRegion = num;}
	  Ped::Ttree* getTree() const { return tree;}
	  Ped::Ttree *tree;
	  bool getDrawTree() const {return Ped::Model::drawTree;}
	  void enableDrawTree() {Ped::Model::drawTree = true;}
  private:
	  static bool rebuild;
	  static bool prune;
	  static bool test;
	  static bool drawTree;
	  static int agentsPerRegion;
    IMPLEMENTATION implementation;   
    std::vector<Tagent*> agents;
    void doSafeMovement( Ped::Tagent *agent);
    // The maximum quadtree depth
    static const int treeDepth;

    size_t numThreads;

    // Keeps track of the positions of each agent


    // Maps the agent to the tree node containing it. Convenience data structure
    // in order to update the tree in case the agent moves.
    std::map<const Ped::Tagent*, Ped::Ttree*> *treehash;

    // Returns the set of neighboring agents for the specified position
	  set<const Ped::Tagent*> getNeighbors(int x, int y, int dist) const;
	  void getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int d) const;
  };
}
#endif
