#ifndef __DSTAR_H__
#define __DSTAR_H__

#include<vector>
#include "globalVariables.h"
#include "pqueue.h"

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class dstar { 

public:
	dstar(int rows_, int cols_);
	~dstar();
	void runDstar(int startX, int startY, int goalX, int goalY);


	friend void copyMazeToDisplayMap(GridWorld &gWorld, dstar* ds);
	friend void copyDisplayMapToMaze(GridWorld &gWorld, dstar* ds);
private:
	vector<vector<dStarNode> > maze;
	pqueue U;
	dStarNode *start; 
	dStarNode *goal;

	double km;

	int rows; 
	int cols;

	void initialise(int startX, int startY, int goalX, int goalY);
	void updateVertex(dStarNode s);	
	void computeShortestPath();

	bool smallerKey(double *a, double*b);
	bool shouldLoop();
	void calcKey(dStarNode *node);
	int maxValue(int v1, int v2);
	double minValue(double g_, double rhs_);
	double calc_H(int x, int y);
	void updateHValues();
};

#endif