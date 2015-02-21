#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_agent.h"
#include <CL/cl.h>

namespace Ped{
  enum IMPLEMENTATION {OPENCL, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
	public:
      void setup(std::vector<Tagent*> agentsInScenario, IMPLEMENTATION imp);
	  void tick();
	  const std::vector<Tagent*> getAgents() const;
	  void setNumThreads(size_t threads);
	private:
	  IMPLEMENTATION implementation;
	  std::vector<Tagent*> agents;
	  size_t numThreads;
          float *tempagents;
          float *waypoints;
          char *reached;
          cl_context context;

          int sizeA, sizeW, sizeR;
 
          cl_command_queue command_queue;
 
          cl_mem agents_mem_obj;
          cl_mem waypoints_mem_obj;
          cl_mem reached_mem_obj;
          cl_program program;
          cl_platform_id platform_id;
          cl_device_id device_id;   
          cl_uint ret_num_devices;
          cl_uint ret_num_platforms;
          cl_int ret;
          cl_kernel kernel;
          size_t global_item_size;
          size_t local_item_size;
  };
}
#endif
