#ifndef __ASTARSEARCH_H__
#define __ASTARSEARCH_H__

#include <iostream>
#include <stdio.h>
#include <vector>
#include <malloc.h>
#include <assert.h>
#include "globalVariables.h"


#define DISTANCE_CALC 0 //1 = Euclidean : 0 = Manhattan

class GridWorld;

class AstarSearch { 
public: 
	
	AstarSearch(int rows_, int cols_);
	~AstarSearch();
	
	friend void copyMazeToDisplayMap(GridWorld &gWorld, AstarSearch* astar);
	friend void copyDisplayMapToMaze(GridWorld &gWorld, AstarSearch* astar);

	void initialise(int startX, int startY, int goalX, int goalY);
	void printMaze();
	void printNode(AstarCell *a);

	vector<AstarCell> computeShortestPath(int &numOfVertexExpansions, int &maxQlen, int &vertexAccesses);
	
private: 
	vector<vector<AstarCell> > maze;
	vector<AstarCell> open, closed;

	vector<AstarCell> final_path;

	AstarCell * start; 
	AstarCell * goal;

	int rows; 
	int cols;

	//The g costs based on which direction
	const double diag = sqrt(2);
	const double cost[8] = {
		diag, 1, diag,
		1      , 1,
		diag, 1, diag
	};

	int maxValue(int v1, int v2); 
	double calculateH_euclid(int x, int y);
	double calculateH_manhat(int x, int y);
	double calculateFCost(const AstarCell a);
	AstarCell lowestFCostInOpen();
	bool isGoal(AstarCell a);
	void generateChildNodes(AstarCell &node);
	int inOpenArray(const AstarCell *a);
	int inClosedArray(const AstarCell *a);
};


#endif
