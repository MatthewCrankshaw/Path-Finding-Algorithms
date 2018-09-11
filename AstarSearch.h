#ifndef __ASTARSEARCH_H__
#define __ASTARSEARCH_H__

#include <iostream>
#include <stdio.h>
#include <vector>
#include <malloc.h>
#include <assert.h>
#include "globalVariables.h"

class GridWorld;

//extern int goal_x, goal_y;
//extern int start_x, start_y;

//~ extern int numberOfExpandedStates;
//extern int numberOfVertexAccesses;

class AstarSearch { 
public: 
	
	AstarSearch(int rows_, int cols_);
	~AstarSearch();
	
	friend void copyMazeToDisplayMap(GridWorld &gWorld, AstarSearch* astar);
	friend void copyDisplayMapToMaze(GridWorld &gWorld, AstarSearch* astar);

	void initialise(int startX, int startY, int goalX, int goalY);
	void printMaze();

	bool computeShortestPath(int &numOfVertexExpansions, int &maxQlen, int &vertexAccesses);
	
private: 
	vector<vector<AstarCell> > maze;
	vector<AstarCell> open, closed;

	AstarCell * start; 
	AstarCell * goal;

	int rows; 
	int cols;


	int maxValue(int v1, int v2); 
	double calculateH_euclid(int x, int y);
	double calculateH_manhat(int x, int y);
	double calculateFCost(const AstarCell a);
	AstarCell lowestFCostInOpen();
	bool isGoal(AstarCell a);
	void generateChildNodes(AstarCell &node);
	int inOpenArray(const AstarCell *a);
};


#endif
