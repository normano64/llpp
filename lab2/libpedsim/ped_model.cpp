#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <omp.h>
#include <iostream>
#include <CL/cl.h>
#include <iostream>

const char *kernelsource = "                                               \
  __kernel void go(__global double *agentX,                                \
                   __global double *agentY,                                \
                   __global double *waypointX,                             \
                   __global double *waypointY,                             \
                   __global double *waypointR,                             \
                   __global bool *waypointRad) {                           \
    int i = get_global_id(0);                                              \
    double waypointXn = waypointX[i] - agentX[i];                          \
    double waypointYn = waypointY[i] - agentY[i];                          \
    double length = sqrt(waypointXn*waypointXn + waypointYn*waypointYn);   \
    if(length < waypointR[i]) {                                            \
        waypointRad[i] = true;                                             \
    } else {                                                               \
        waypointRad[i] = false;                                            \
    }                                                                      \
    waypointXn /= length;                                                  \
    waypointYn /= length;                                                  \
    agentX[i] = round(agentX[i] + waypointXn);                             \
    agentY[i] = round(agentY[i] + waypointYn);                             \
  }";

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION impl)
{
    agents = agentsInScenario;
    implementation = impl;

    if(implementation == OPENCL) {
        size_t length = agents.size();

        tempagentsX = new double[length];
        tempagentsY = new double[length];
        waypointX = new double[length];
        waypointY = new double[length];
        waypointR = new double[length];
        waypointRad = new bool[length];
        
        for(int i = 0; i < length; i++) {
            tempagentsX[i] = agents[i]->getX();
            tempagentsY[i] = agents[i]->getY();
            agents[i]->destination = agents[i]->getNextDestination();
            waypointX[i] = agents[i]->destination->getx();
            waypointY[i] = agents[i]->destination->gety();
            waypointR[i] = agents[i]->destination->getr();
        }
        
        int size = length * sizeof(double);
        int sizeB = length * sizeof(bool);
        
        platform_id = NULL;
        device_id = NULL;   
        ret_num_devices;
        ret_num_platforms;
        ret = clGetPlatformIDs(1, &platform_id, &ret_num_platforms);
        ret = clGetDeviceIDs(platform_id, CL_DEVICE_TYPE_DEFAULT, 1, &device_id, &ret_num_devices);

        context = clCreateContext( NULL, 1, &device_id, NULL, NULL, &ret);
 
        command_queue = clCreateCommandQueue(context, device_id, 0, &ret);
 
        agentX_mem_obj = clCreateBuffer(context, CL_MEM_READ_WRITE, size, NULL, &ret);
        agentY_mem_obj = clCreateBuffer(context, CL_MEM_READ_WRITE, size, NULL, &ret);
        waypointX_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, size, NULL, &ret);
        waypointY_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, size, NULL, &ret);
        waypointR_mem_obj = clCreateBuffer(context, CL_MEM_READ_ONLY, size, NULL, &ret);
        waypointRad_mem_obj = clCreateBuffer(context, CL_MEM_WRITE_ONLY, sizeB, NULL, &ret);

        ret = clEnqueueWriteBuffer(command_queue, agentX_mem_obj, CL_TRUE, 0, size, tempagentsX, 0, NULL, NULL);
        ret = clEnqueueWriteBuffer(command_queue, agentY_mem_obj, CL_TRUE, 0, size, tempagentsY, 0, NULL, NULL);
        ret = clEnqueueWriteBuffer(command_queue, waypointX_mem_obj, CL_TRUE, 0, size, waypointX, 0, NULL, NULL);
        ret = clEnqueueWriteBuffer(command_queue, waypointY_mem_obj, CL_TRUE, 0, size, waypointY, 0, NULL, NULL);
        ret = clEnqueueWriteBuffer(command_queue, waypointR_mem_obj, CL_TRUE, 0, size, waypointR, 0, NULL, NULL);
        ret = clEnqueueWriteBuffer(command_queue, waypointRad_mem_obj, CL_TRUE, 0, sizeB, waypointRad, 0, NULL, NULL);

        program = clCreateProgramWithSource(context, 1, (const char **)&kernelsource, NULL, &ret);

        ret = clBuildProgram(program, 1, &device_id, NULL, NULL, NULL);
        if(ret != CL_SUCCESS) {
            std::cout << "error: " << ret << std::endl;
        }

        kernel = clCreateKernel(program, "go", &ret);

        ret = clSetKernelArg(kernel, 0, sizeof(cl_mem), (void *)&agentX_mem_obj);
        ret = clSetKernelArg(kernel, 1, sizeof(cl_mem), (void *)&agentY_mem_obj);
        ret = clSetKernelArg(kernel, 2, sizeof(cl_mem), (void *)&waypointX_mem_obj);
        ret = clSetKernelArg(kernel, 3, sizeof(cl_mem), (void *)&waypointY_mem_obj);
        ret = clSetKernelArg(kernel, 4, sizeof(cl_mem), (void *)&waypointR_mem_obj);
        ret = clSetKernelArg(kernel, 5, sizeof(cl_mem), (void *)&waypointRad_mem_obj);

        global_item_size = length;
        local_item_size = 100;

       
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
        std::cout << "ost " << agents[1]->getX() << "," << agents[1]->getY() << std::endl;
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
        size_t global_item_size = length;
        size_t local_item_size = 100;
        
        int size = length * sizeof(double);
        int sizeB = length * sizeof(bool);
        ret = clEnqueueNDRangeKernel(command_queue, kernel, 1, NULL, &global_item_size, &local_item_size, 0, NULL, NULL);

        ret =  clFinish(command_queue);

        ret = clEnqueueReadBuffer(command_queue, agentX_mem_obj, CL_TRUE, 0, size, tempagentsX, 0, NULL, NULL);
        ret = clEnqueueReadBuffer(command_queue, agentY_mem_obj, CL_TRUE, 0, size, tempagentsY, 0, NULL, NULL);
        ret = clEnqueueReadBuffer(command_queue, waypointRad_mem_obj, CL_TRUE, 0, sizeB, waypointRad, 0, NULL, NULL);

        bool update;
        std::cout << "ost " << tempagentsX[1] << "," << tempagentsY[1] << std::endl;
        for(unsigned int i = 0; i < length; i++) {
            agents[i]->setX(tempagentsX[i]);
            agents[i]->setY(tempagentsY[i]);
            if(waypointRad[i] == true) {
                agents[i]->waypoints.push_back(agents[i]->destination);
                agents[i]->destination = agents[i]->getNextDestination();
                
                waypointX[i] = agents[i]->destination->getx();
                waypointY[i] = agents[i]->destination->gety();
                waypointR[i] = agents[i]->destination->getr();
                update = true;
            }
        }
        if(update) {
            ret = clEnqueueWriteBuffer(command_queue, waypointX_mem_obj, CL_TRUE, 0, size, waypointX, 0, NULL, NULL);
            ret = clEnqueueWriteBuffer(command_queue, waypointY_mem_obj, CL_TRUE, 0, size, waypointY, 0, NULL, NULL);
            ret = clEnqueueWriteBuffer(command_queue, waypointR_mem_obj, CL_TRUE, 0, size, waypointR, 0, NULL, NULL);
        }
        
        break;
    }
}

