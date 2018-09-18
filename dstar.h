#ifndef __DSTAR_H__
#define __DSTAR_H__

#include<vector>
#include "globalVariables.h"
#include "pqueue.h"

#define DISTANCE_CALC 0 // 1 = euclid, 0 = manhattan

class GridWorld; //forward declare class GridWorld to be able to create the friend functions later

class dstar { 

public:
	dstar(int rows_, int cols_);
	~dstar();
	void runDstar(int startX, int startY, int goalX, int goalY);
	unsigned getMaxQLen();

	friend void copyMazeToDisplayMap(GridWorld &gWorld, dstar* ds);
	friend void copyDisplayMapToMaze(GridWorld &gWorld, dstar* ds);
private:
	vector<vector<dStarNode> > maze;
	pqueue U;
	dStarNode *start; 
	dStarNode *goal;
	int numExpan, maxQLen;

	double km;

	int rows; 
	int cols;

	const double diag = sqrt(2);
	const double cost[8] = {
		diag, 1, diag,
		1      , 1,
		diag, 1, diag
	};

	void initialise(int startX, int startY, int goalX, int goalY);
	void updateVertex(dStarNode s);	
	void computeShortestPath();

	bool smallerKey(double *a, double*b);
	bool shouldLoop();
	void calcKey(dStarNode *node);
	int maxValue(int v1, int v2);
	double minValue(double g_, double rhs_);
	double calc_H_euclid(int x, int y);
	double calc_H_manhat(int x, int y);
	void updateHValues();
};

#endif