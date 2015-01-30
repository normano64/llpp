#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <omp.h>
#include <iostream>


void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario)
{
  agents = agentsInScenario;
  implementation = OMP;
}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

void Ped::Model::tick()
{
  std::vector<Ped::Tagent*> agents = this->getAgents();
  size_t length = agents.size();
  switch(implementation)
  {
	case SEQ:
	  for(int i = 0;i < length;i++) 
	  {
		agents[i]->whereToGo();
		agents[i]->go();
	  }
	  break;
	case OMP:
#pragma omp parallel for shared (agents, length) private (i) schedule(auto)
	  for(int i = 0; i < length; i++)
	  {
		agents[i]->whereToGo();
		agents[i]->go();
	  }
	  break;
	case PTHREAD:
	  break;
  }
}

