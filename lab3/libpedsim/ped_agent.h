//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//
#ifndef _ped_agent_h_
#define _ped_agent_h_ 1


// g++ does not understand cuda stuff. This makes it ignore them. (use this if you want)
#ifndef __CUDACC__
#define __device__ 
#define __host__
#endif

#include "ped_vector.h"
#include <vector>
#include <deque>

using namespace std;

namespace Ped {
  class Twaypoint;

  class Tagent {
  public:
    Tagent(int posX, int posY);
    Tagent(double posX, double posY);

    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////
    
    // Returns the desired positons coordinates
    int getDesiredX() const {return desiredPosition.x;}
    int getDesiredY() const {return desiredPosition.y;}

    ////////////
    /// END NEW 
    ///////////////////////////////////////////////

    // Sets the agent's position
    void setX(int x) {position.x = x;}
    void setY(int y) {position.y = y;}

    // Computes forces that determine the next position
    Tvector whereToGo();

    // Update the position according to computed forces
    void go();

    const Tvector& getPosition() const { return position; }
    int getX() const { return position.x; };
    int getY() const { return position.y; };

    void addWaypoint(Twaypoint* wp);
    bool removeWaypoint(const Twaypoint* wp);
    void clearWaypoints();
    void setWaypointBehavior(int mode) { waypointBehavior = mode; };

    bool reachedDestination() { return (destination == NULL); };
    
  private:
    Tagent() {};
    
    // The current position
    Tvector position;

    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////

    // The desired next position
    Tvector desiredPosition;

    ////////////
    /// END NEW
    ///////////////////////////////////////////////

    Twaypoint* destination;
    Twaypoint* lastDestination;

    deque<Twaypoint*> waypoints;
    int waypointBehavior;

    // The force towards the current destination
    Tvector waypointForce;
    
    // Interntal init function 
    void init(int posX, int posY);

    Twaypoint* getNextDestination();
    Twaypoint* getNextWaypoint();
  };
}

#endif
