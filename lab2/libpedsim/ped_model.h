#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"

namespace Ped{
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
	public:
	  void setup(std::vector<Tagent*> agentsInScenario);
	  void tick();
	  const std::vector<Tagent*> getAgents() const;
	  void setImplementation(IMPLEMENTATION imp);
	  void setNumThreads(size_t threads);
	private:
	  IMPLEMENTATION implementation;
	  std::vector<Tagent*> agents;
	  size_t numThreads;
  };
}
#endif
