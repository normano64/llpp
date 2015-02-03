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

struct updateArgs
{
	size_t start;
	size_t end;
	std::vector<Ped::Tagent*> agents;
};

void* update(void* args)
{
  updateArgs arg = *((updateArgs*)args);
  size_t start = arg.start;
  size_t end = arg.end;
  std::vector<Ped::Tagent*> agents = arg.agents;
  for(size_t i = start; i < end; i++)
  {
	agents[i]->whereToGo();
	agents[i]->go();
  }
  pthread_exit(NULL);
}

void Ped::Model::setImplementation(IMPLEMENTATION impl)
{
  implementation = impl;
}

void Ped::Model::tick()
{
  std::vector<Ped::Tagent*> agents = this->getAgents();
  size_t length = agents.size();
  size_t nthreads = (MAXTHREADS < length) ? MAXTHREADS : length;
  pthread_t threads[nthreads];
  size_t chunk = length / nthreads;
  updateArgs args[nthreads];
  size_t start = 0;
  size_t end = chunk;
  pthread_attr_t attr;
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
	  pthread_attr_init(&attr);
	  pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	  for(size_t i = 0; i < nthreads; i++)
	  {
	    args[i].start = start;
		args[i].end = end;
		args[i].agents = agents;
		start = end;
		end = (end+chunk < agents.size()) ? end+chunk : agents.size();
		pthread_create(&threads[i], &attr, update, (void*)(&args[i]));
	  }
	  pthread_attr_destroy(&attr);
	  void* status;
	  for(size_t i = 0; i < nthreads; i++)
		pthread_join(threads[i], &status);
	  break;
  }
}

