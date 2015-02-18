#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <omp.h>
#include <iostream>
#include <CL/cl.h>
#include <iostream>

const char *kernelsource = "                                               \
  __kernel void go(__global double *agents,                                \
                   __global double *waypoints) {                           \
    int i = get_global_id(0) * 3;                                          \
    double waypointX = waypoints[i] - agents[i];                           \
    double waypointY = waypoints[i+1] - agents[i+1];                       \
    double length = sqrt(waypointX*waypointX + waypointY*waypointY);       \
    if(length < waypoints[i+2]) {                                          \
        agents[i+2] = 1;                                                   \
    } else {                                                               \
        agents[i+2] = 0;                                                   \
    }                                                                      \
    if(length == .0) { \
        waypointX = 0; \
        waypointY = 0; \
    } else { \
        waypointX /= length;                                                   \
        waypointY /= length;                                                   \
    } \
    agents[i] = round(agents[i] + waypointX);                              \
    agents[i+1] = round(agents[i+1] + waypointY);                          \
  }";

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION impl)
{
    agents = agentsInScenario;
    implementation = impl;

    if(implementation == OPENCL) {
        size_t length = agents.size();

        // tempagentsX = new double[length];
        tempagents = new double[length*3];
        waypoints = new double[length*3];
        //waypointY = new double[length];
        // waypointR = new double[length];
        //waypointRad = new bool[length];
        size_t ind = 0;
        for(int i = 0; i < length; i++) {
          ind = i*3;
          agents[i]->destination = agents[i]->getNextDestination();
          tempagents[ind] = agents[i]->getX();
          tempagents[ind+1] = agents[i]->getY();
          tempagents[ind+2] = .0;
          waypoints[ind] = agents[i]->destination->getx();
          waypoints[ind+1] = agents[i]->destination->gety();
          waypoints[ind+2] = agents[i]->destination->getr();
            // tempagentsX[i] = agents[i]->getX();
            // tempagentsY[i] = agents[i]->getY();
            // agents[i]->destination = agents[i]->getNextDestination();
            // waypointX[i] = agents[i]->destination->getx();
            // waypointY[i] = agents[i]->destination->gety();
            // waypointR[i] = agents[i]->destination->getr();
        }
        
        size = length * sizeof(double) * 3;
        global_item_size = length;
        local_item_size = 100;
        
        platform_id = NULL;
        device_id = NULL;
        ret = clGetPlatformIDs(1, &platform_id, &ret_num_platforms);
        ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);

        context = clCreateContext( NULL, 1, &device_id, NULL, NULL, &ret);
 
        command_queue = clCreateCommandQueue(context, device_id, 0, &ret);
 
        // agentX_mem_obj = clCreateBuffer(context, CL_MEM_READ_WRITE, size, NULL, &ret);
        // agentY_mem_obj = clCreateBuffer(context, CL_MEM_READ_WRITE, size, NULL, &ret);
        agents_mem_obj = clCreateBuffer(context, CL_MEM_READ_WRITE, size, NULL, &ret);
        waypoints_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, size, NULL, &ret);

        // ret = clEnqueueWriteBuffer(command_queue, agentX_mem_obj, CL_TRUE, 0, size, tempagentsX, 0, NULL, NULL);
        // ret = clEnqueueWriteBuffer(command_queue, agentY_mem_obj, CL_TRUE, 0, size, tempagentsY, 0, NULL, NULL);
        ret = clEnqueueWriteBuffer(command_queue, agents_mem_obj, CL_TRUE, 0, size, tempagents, 0, NULL, NULL);
        ret = clEnqueueWriteBuffer(command_queue, waypoints_mem_obj, CL_TRUE, 0, size, waypoints, 0, NULL, NULL);

        program = clCreateProgramWithSource(context, 1, (const char **)&kernelsource, NULL, &ret);

        ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);
        if(ret != CL_SUCCESS) {
            std::cout << "error: " << ret << std::endl;
        }

        kernel = clCreateKernel(program, "go", &ret);

        ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&agents_mem_obj);
        ret = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&waypoints_mem_obj);
    }
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

void Ped::Model::setNumThreads(size_t threads)
{
    numThreads = threads;
}

void Ped::Model::tick() {
    std::vector<Ped::Tagent*> agents = this->getAgents();
    size_t length = agents.size();
    size_t nthreads = (numThreads < length) ? numThreads : length;
    pthread_t threads[nthreads];
    size_t chunk = length / nthreads;
    updateArgs args[nthreads];
    size_t start = 0;
    size_t end = chunk;
    pthread_attr_t attr;
    switch(implementation) {
    case SEQ:
        for(int i = 0;i < length;i++) {
            agents[i]->whereToGo();
            agents[i]->go();
        }
        //std::cout << "ost " << agents[1]->getX() << "," << agents[1]->getY() << std::endl;
        break;
    case OMP:
        omp_set_dynamic(0);
        omp_set_num_threads(nthreads);
#pragma omp parallel for shared (agents, length) private (i) schedule(auto)
        for(int i = 0; i < length; i++) {
            agents[i]->whereToGo();
            agents[i]->go();
        }
        break;
    case PTHREAD:
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
        for(size_t i = 0; i < nthreads; i++) {
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
    case OPENCL:
        ret = clEnqueueNDRangeKernel(command_queue, kernel, 1, NULL, &global_item_size, &local_item_size, 0, NULL, NULL);

        ret =  clFinish(command_queue);

        // ret = clEnqueueReadBuffer(command_queue, agentX_mem_obj, CL_TRUE, 0, size, tempagentsX, 0, NULL, NULL);
        // ret = clEnqueueReadBuffer(command_queue, agentY_mem_obj, CL_TRUE, 0, size, tempagentsY, 0, NULL, NULL);
        ret = clEnqueueReadBuffer(command_queue, agents_mem_obj, CL_TRUE, 0, size, tempagents, 0, NULL, NULL);

        bool update;
        size_t ind;
        // std::cout << "ost " << tempagentsX[1] << "," << tempagentsY[1] << std::endl;
        for(unsigned int i = 0; i < length; i++) {
          ind = i * 3;
            agents[i]->setX(tempagents[ind]);
            agents[i]->setY(tempagents[ind+1]);
            if(agents[ind+2] != 0) {
                agents[i]->waypoints.push_back(agents[i]->destination);
                agents[i]->destination = agents[i]->getNextWaypoint();

                waypoints[ind] = agents[i]->destination->getx();
                waypoints[ind+1] = agents[i]->destination->gety();
                waypoints[ind+2] = agents[i]->destination->getr();
                update = true;
            }
        }
        if(update) {
            ret = clEnqueueWriteBuffer(command_queue, waypoints_mem_obj, CL_TRUE, 0, size, waypoints, 0, NULL, NULL);
        }
        break;
    }
}

