#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <iostream>


void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario)
{
  agents = agentsInScenario;
  implementation = SEQ;
}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

void Ped::Model::tick()
{
  // EDIT HERE
  std::vector<Ped::Tagent*> agents = this->getAgents();
  size_t length = agents.size();
  for(int i = 0;i < length;i++) {
    agents[i]->whereToGo();
  }
  for(int i = 0;i < length;i++) {
    agents[i]->go();
  }
  std::cout << "test" << std::endl;
}

