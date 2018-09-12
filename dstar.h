#ifndef __DSTAR_H__
#define __DSTAR_H__

#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow
#include<iostream>
#include<vector>
#include "globalVariables.h"
#include "gridworld.h"
#include "pqueue.h"

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class dstar { 

public:
	dstar(int rows_, int cols_);
	~dstar();
	void initialise(int startX, int startY, int goalX, int goalY);
	void updateVertex();	
	void computeShortestPath();
	void runDstar();

private:
	vector<vector<dStarNode> > maze;
	pqueue U;
	dStarNode *start; 
	dStarNode *goal;

	double km;

	int rows; 
	int cols;

};

#endif