//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
#include "ped_tree.h"

#include <cassert>
#include <cstddef>
#include <algorithm>
#include <iostream>
#include <omp.h>
using namespace std;

#define INSIDE(_x, _y, x, y, w, h) (_x >= x && _x <= x+w && _y >= y && _y <= y+h)

/// Description: set intial values
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::Ttree(Ped::Ttree *root,std::map<const Ped::Tagent*, Ped::Ttree*> *treehash, int pdepth, int pmaxDepth, double px, double py, double pw, double ph, int mAgents) {
  // more initializations here. not really necessary to put them into the initializator list, too.
  this->root = root != NULL ? root: this;
  this->treehash = treehash;
  isleaf = true;
  x = px;
  y = py;
  w = pw;
  h = ph;
  depth = pdepth;
  maxDepth = pmaxDepth;
  tree1 = NULL;
  tree2 = NULL;
  tree3 = NULL;
  tree4 = NULL;
  maxAgents = mAgents;
};


/// Destructor. Deleted this node and all its children. If there are any agents left, they are removed first (not deleted).
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::~Ttree() {
  clear();
}


void Ped::Ttree::clear() {
  if(isleaf) 
  {
	agents.clear();
  }
  else 
  {
	tree1->clear();
	delete tree1;
	tree2->clear();
	delete tree2;
	tree3->clear();
	delete tree3;
	tree4->clear();
	delete tree4;
	isleaf = true;
  }
}

bool cmp(const Ped::Tagent *a, const Ped::Tagent *b) {
  return a->getX() == b->getX() && a->getY() == b->getY();
}

/// Adds an agent to the tree. Searches the right node and adds the agent there.
/// If there are too many agents at that node allready, a new child is created.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a The agent to add
void Ped::Ttree::addAgent(const Ped::Tagent *a) 
{
  if (isleaf) 
  {
	agents.insert(a);
	//model->setResponsibleTree(this, a);
	(*treehash)[a] = this;
  }
  else 
  {
	double _x = a->getX();
    double _y = a->getY();
	if (tree3->inside(_x,_y))//(a->getX() >= x+w/2) && (a->getY() >= y+h/2)) 
	{
	  tree3->addAgent(a); // 3
	} 
	else if (tree1->inside(_x,_y))
	{
	  tree1->addAgent(a); // 1
	}
	else if (tree2->inside(_x,_y)) 
	{
	  tree2->addAgent(a); // 2
	} 
	else if (tree4->inside(_x,_y)) 
	{
	  tree4->addAgent(a); // 4
	}
	else
	{
	  std::cout << "Oh-oh, we have an oopsie!\t x:" << _x << " - " << x << "\t y: " << _y << " - " << y << "\t w: " << w << "\t h" << h << std::endl;
	}
  }
  
  if (agents.size() > maxAgents && depth < maxDepth) 
  {
	isleaf = false;
	addChildren();
	const Ped::Tagent *a = NULL;
	double _x, _y;
	while (!agents.empty()) 
	{
	  a = (*agents.begin());
	  _x = a->getX();
	  _y = a->getY();
	  if (tree3->inside(_x,_y)) 
	  {
		tree3->addAgent(a); // 3
	  } 
	  else if (tree1->inside(_x,_y)) 
	  {
		tree1->addAgent(a); // 1
	  } 
	  else if (tree2->inside(_x,_y)) 
	  {
		tree2->addAgent(a); // 2
	  } 
	  else if (tree4->inside(_x,_y)) 
	  {
		tree4->addAgent(a); // 4
	  }
	  else
	  {
		std::cout << "Another oopsie" << std::endl;
	  }
	  agents.erase(a);
	}
  }
}


/// A little helper that adds child nodes to this node
/// \author  chgloor
/// \date    2012-01-28
void Ped::Ttree::addChildren() {
  tree1 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x, y, w/2.0, h/2.0, maxAgents);
  tree2 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x+w/2.0, y, w/2.0, h/2.0, maxAgents);
  tree3 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x+w/2.0, y+h/2.0, w/2.0, h/2.0, maxAgents);
  tree4 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x, y+h/2.0, w/2.0, h/2.0, maxAgents);
}


Ped::Ttree* Ped::Ttree::getChildByPosition(double xIn, double yIn) const {
  if(tree1->inside(xIn, yIn))
	return tree1;
  if(tree2->inside(xIn, yIn))
	return tree2;
  if(tree3->inside(xIn, yIn))
	return tree3;
  if(tree4->inside(xIn, yIn))
	return tree4;

  // this should never happen
  return NULL;
}


/// Updates the tree structure if an agent moves. Removes the agent and places it again, if outside boundary.
/// If an this happens, this is O(log n), but O(1) otherwise.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a the agent to update
void Ped::Ttree::moveAgent(const Ped::Tagent *a) {
  if (!inside(a->getX(), a->getY()))
  {//(a->getX() < x) || (a->getX() > (x+w)) || (a->getY() < y) || (a->getY() > (y+h))) {
	agents.erase(a);
	root->addAgent(a);
  }
}


bool Ped::Ttree::removeAgent(const Ped::Tagent *a) {
  if(isleaf) {
	size_t removedCount = agents.erase(a);
	return (removedCount > 0);
  }
  else {
	return getChildByPosition(a->getX(), a->getY())->removeAgent(a);
  }
}

/// Checks if this tree node has not enough agents in it to justify more child nodes. It does this by checking all
/// child nodes, too, recursively. If there are not enough children, it moves all the agents into this node, and deletes the child nodes.
/// \author  chgloor
/// \date    2012-01-28
/// \return  the number of agents in this and all child nodes.
int Ped::Ttree::cut() {
  if (isleaf)
	return agents.size();

  int count = 0;
  count += tree1->cut();
  count += tree2->cut();
  count += tree3->cut();
  count += tree4->cut();
  if (count < maxAgents/2) {
	assert(tree1->isleaf == true);
	assert(tree2->isleaf == true);
	assert(tree3->isleaf == true);
	assert(tree4->isleaf == true);
	agents.insert(tree1->agents.begin(), tree1->agents.end());
	agents.insert(tree2->agents.begin(), tree2->agents.end());
	agents.insert(tree3->agents.begin(), tree3->agents.end());
	agents.insert(tree4->agents.begin(), tree4->agents.end());
	isleaf = true;
	for (set<const Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it) {
	  const Tagent *a = (*it);
	  (*treehash)[a] = this;
	}
	delete tree1; tree1 = NULL;
	delete tree2; tree2 = NULL;
	delete tree3; tree3 = NULL;
	delete tree4; tree4 = NULL;
  }
  return count;
}


/// Returns the set of agents that is stored within this tree node
/// \author  chgloor
/// \date    2012-01-28
/// \return  The set of agents
/// \todo This might be not very efficient, since all childs are checked, too. And then the set (set of pointer, but still) is being copied around.
set<const Ped::Tagent*> Ped::Ttree::getAgents() const {
  if (isleaf)
	return agents;

  set<const Ped::Tagent*> ta;
  set<const Ped::Tagent*> t1 = tree1->getAgents();
  set<const Ped::Tagent*> t2 = tree2->getAgents();
  set<const Ped::Tagent*> t3 = tree3->getAgents();
  set<const Ped::Tagent*> t4 = tree4->getAgents();
  ta.insert(t1.begin(), t1.end());
  ta.insert(t2.begin(), t2.end());
  ta.insert(t3.begin(), t3.end());
  ta.insert(t4.begin(), t4.end());
  return ta;
}

void Ped::Ttree::getAgents(list<const Ped::Tagent*>& outputList) const {
  if(isleaf) {
	for (set<const Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it) {
	  const Ped::Tagent* currentAgent = (*it);
	  outputList.push_back(currentAgent);
	}
  }
  else {
	tree1->getAgents(outputList);
	tree2->getAgents(outputList);
	tree3->getAgents(outputList);
	tree4->getAgents(outputList);
  }
}


/// Checks if a point x/y is within the space handled by the tree node, or within a given radius r
/// \author  chgloor
/// \date    2012-01-29
/// \return  true if the point is within the space
/// \param   px The x co-ordinate of the point
/// \param   py The y co-ordinate of the point
/// \param   pr The radius
bool Ped::Ttree::intersects(double px, double py, double pr) const {
  if (((px+pr) >= x) && ((px-pr) <= (x+w)) && ((py+pr) >= y) && ((py-pr) <= (y+h)))
	return true; // x+-r/y+-r is inside
  else
	return false;
}

bool Ped::Ttree::inside(double px, double py)
{
  //cout << px << ">=" <<  x << "&&" << px << "<=" << x+w << "&&" << py << ">=" << y << "&&" << py << "<=" << y+h << endl;
  return (px >= x && px < x+w && py >= y && py < y+h);
}


void Ped::Ttree::_doSafeMovement(std::vector<set<const Ped::Tagent*> >& betweenRegions, int tid)
{
  if(!isleaf)
  {
	tree1->_doSafeMovement(betweenRegions, tid);
	tree2->_doSafeMovement(betweenRegions, tid);
	tree3->_doSafeMovement(betweenRegions, tid);
	tree4->_doSafeMovement(betweenRegions, tid);
  }
  else
  {
	std::set<const Ped::Tagent*>::iterator it1;
	std::set<const Ped::Tagent*>::iterator it2;
	std::vector<std::pair<int, int> > takenPositions;
	Ped::Tagent* agent = NULL;
	std::vector<std::pair<int, int> > prioritizedAlternatives;
	std::pair<int, int> position;
	std::pair<int, int> oldPos;
	std::pair<int, int> pDesired;
	std::pair<int, int> p1, p2;
	int diffX, diffY;
	std::vector<pair<int, int> >::iterator it;
	std::set<const Ped::Tagent*> _agents = agents;
	std::map<const Ped::Tagent*, bool> erased;
	int tid = omp_get_thread_num();
	for(it1 = _agents.begin(); it1 != _agents.end(); it1++)
	{
	  position = std::make_pair((*it1)->getX(), (*it1)->getY());
	  takenPositions.push_back(position);
	}
	for(it1 = _agents.begin(); it1 != _agents.end(); it1++)
	{
	  prioritizedAlternatives.clear();
	  agent = const_cast<Tagent*>(*it1);
	  oldPos = std::make_pair(agent->getX(), agent->getY());

	  pDesired = std::make_pair(agent->getDesiredX(), agent->getDesiredY());
	  prioritizedAlternatives.push_back(pDesired);

	  diffX = pDesired.first - agent->getX();
	  diffY = pDesired.second - agent->getY();
	  if (diffX == 0 || diffY == 0) {
		// Agent wants to walk straight to North, South, West or East
		p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
		p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
	  } else {
		// Agent wants to walk diagonally
		p1 = std::make_pair(pDesired.first, agent->getY());
		p2 = std::make_pair(agent->getX(), pDesired.second);
	  }
          
	  prioritizedAlternatives.push_back(p1);
	  prioritizedAlternatives.push_back(p2);
	  // Find the first empty alternative position
	  for (it = prioritizedAlternatives.begin(); it != prioritizedAlternatives.end(); ++it) {
		// If the current position is not yet taken by any neighbor
		if(!inside((*it).first, (*it).second))
		{
		  betweenRegions[tid].insert(agent);		
		  break;
		}
		if (std::find(takenPositions.begin(), takenPositions.end(), *it) == takenPositions.end()) {
		  // Set the agent's position
		  agent->setX((*it).first);
		  agent->setY((*it).second);
		  std::vector<std::pair<int, int> >::iterator posIt = std::find(takenPositions.begin(), takenPositions.end(), oldPos);
		  takenPositions.erase(posIt);
		  takenPositions.push_back(std::make_pair(agent->getX(), agent->getY()));
//		  this->moveAgent(agent);
		  break;
		}
	  }
	}
  }
}

void Ped::Ttree::doSafeMovement(std::vector<std::set<const Ped::Tagent*> >&betweenRegions,int depth, int tid) 
{
  // Search for neighboring agents
  if(isleaf)
  {
	_doSafeMovement(betweenRegions, tid);
  }
  else
  {
#pragma omp parallel shared(betweenRegions) private(depth, tid)
	{
#pragma omp single
	  {
#pragma omp task
		{
		  tid = omp_get_thread_num();
		  tree1->doSafeMovement(betweenRegions,depth+1, tid);
		}
#pragma omp task
		{
		  tid = omp_get_thread_num();
		  tree2->doSafeMovement(betweenRegions,depth+1, tid);
		}
#pragma omp task
		{
		  tid = omp_get_thread_num();
		  tree3->doSafeMovement(betweenRegions,depth+1, tid);
		}
#pragma omp task
		{
		  tid = omp_get_thread_num();
		  tree4->doSafeMovement(betweenRegions,depth+1, tid);
		}
	  }
	}
  }
}
	
void Ped::Ttree::prune()
{
  if(!isleaf)
  {
	std::set<const Ped::Tagent*> out;
	getAllAgents(out);
	if(out.size() <= 4 ) // The region is either empty or has less than or equal the amount of agents as the maximum amount and can thus be pruned.
	{
	  isleaf = true;
	  delete tree1; tree1 = NULL;
	  delete tree2; tree2 = NULL;
	  delete tree3; tree3 = NULL;
	  delete tree4; tree4 = NULL;
	  const Ped::Tagent* agent;
	  for(std::set<const Ped::Tagent*>::iterator it = out.begin(); it != out.end(); it++)
	  {
		agent = *it;
		agents.insert(agent);
		(*treehash)[agent] = this;
	  }
	}	
	else // Otherwise, prune each sub-tree.
	{
	  tree1->prune();
	  tree2->prune();
	  tree3->prune();
	  tree4->prune();
	}
  }
}


void Ped::Ttree::getAllAgents(std::set<const Ped::Tagent*>& out)
{
  if(isleaf)
  {
	for(std::set<const Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); it++)
	{
	  out.insert(*it);
	}
  }
  else
  {
	tree1->getAllAgents(out);
	tree2->getAllAgents(out);
	tree3->getAllAgents(out);
	tree4->getAllAgents(out);
  }
}

void Ped::Ttree::_getLeafs(std::set<const Ped::Ttree*> & out)
{
  if(isleaf)
	out.insert(this);
  else
  {
	tree1->_getLeafs(out);
	tree2->_getLeafs(out);
	tree3->_getLeafs(out);
	tree4->_getLeafs(out);
  }
}

std::set<const Ped::Ttree*> Ped::Ttree::getLeafs()
{
  std::set<const Ped::Ttree*> out;
  _getLeafs(out);
  return out;
}

bool Ped::Ttree::occupied(double x, double y) const
{
  for(std::set<const Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); it++)
  {
	if((*it)->getX() == x && (*it)->getY() == y)
	  return true;
  }
  return false;
}
