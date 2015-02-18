//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#include "ped_agent.h"
#include "ped_waypoint.h"
#include "smmintrin.h"
#include "emmintrin.h"

#include <math.h>



Ped::Tagent::Tagent(int posX, int posY) {
    Ped::Tagent::init(posX, posY);
}

Ped::Tagent::Tagent(double posX, double posY) {
    if(Ped::Tvector::VEC) {
        double posv[2];
        posv[0] = posX;
        posv[1] = posY;
        __m128d vec1 = _mm_load_pd(posv);
        vec1 = _mm_round_pd(vec1,_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
        _mm_store_pd(posv, vec1);
        Ped::Tagent::init((int) posv[0], (int) posv[1]);
    } else {
        Ped::Tagent::init((int) round(posX), (int) round(posY));
    }
}

void Ped::Tagent::init(int posX, int posY) {
    position.xyz[0] = posX;
    position.xyz[1] = posY;
    destination = NULL;
}

void Ped::Tagent::go() {
    Tvector moveForce = waypointForce;
    if(Ped::Tvector::VEC) {
        
        __m128d vec1 = _mm_load_pd(position.xyz);
        __m128d vec2 = _mm_load_pd(moveForce.xyz);
        vec1 = _mm_add_pd(vec1, vec2);
        vec1 = _mm_round_pd(vec1,_MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
        _mm_store_pd(position.xyz, vec1);
        
    } else {
        position.xyz[0] = round(position.xyz[0] + moveForce.xyz[0]);
        position.xyz[1] = round(position.xyz[1] + moveForce.xyz[1]);
    }
}

void Ped::Tagent::addWaypoint(Twaypoint* wp) {
    waypoints.push_back(wp);
}

bool Ped::Tagent::removeWaypoint(const Twaypoint* wp) {
    if (destination == wp)
        destination = NULL;
  
    bool removed = false;
    for (int i = waypoints.size(); i > 0; --i) {
        Twaypoint* currentWaypoint = waypoints.front();
        waypoints.pop_front();
        if (currentWaypoint != wp) {
            waypoints.push_back(currentWaypoint);
            removed = true;
        }
    }
  
    return removed;
}

void Ped::Tagent::clearWaypoints() {
    destination = NULL;
  
    for (int i = waypoints.size(); i > 0; --i) {
        waypoints.pop_front();
    }
}

void Ped::Tagent::whereToGo() {
    destination = getNextDestination();

    if (destination == NULL) {
        // Don't move, if nowhere to go
        waypointForce = Ped::Tvector(0, 0, 0);
    }

    bool reachesDestination = false; // if agent reaches destination in n
    waypointForce = destination->getForce(position.xyz[0], position.xyz[1],0,0,&reachesDestination);

    // Circular waypoint chasing
    if (reachesDestination == true) {
        waypoints.push_back(destination);
        destination = NULL;
    }
}

Ped::Twaypoint* Ped::Tagent::getNextDestination() {
    Ped::Twaypoint* nextDestination = NULL;
    if (destination != NULL) {
        nextDestination = destination; // agent hasn't arrived yet
    }
    else if (!waypoints.empty()) {
        nextDestination = getNextWaypoint();
    }
    return nextDestination;
}

Ped::Twaypoint* Ped::Tagent::getNextWaypoint() {
    Twaypoint* waypoint = waypoints.front();
    waypoints.pop_front();
    return waypoint;
}
