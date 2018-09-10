#include <stdio.h>
#include <vector>
#include <malloc.h>
#include <assert.h>
#include <algorithm>    
#include <math.h>

#include "AstarSearch.h"
#include "globalVariables.h"

using namespace std;

/*
// The cell structure is defined in the global variables as below
// is defined in global variables as the main function also needs to access these fields

struct AstarCell
{
	AstarCell *move[DIRECTIONS];
	
	double linkCost[DIRECTIONS]; 
	
	int x, y; 
	double g; 
	double h;
	
	char type;
	
};
*/

AstarSearch::AstarSearch(int rows_, int cols_){
	//ctor
	
	rows = rows_;
	cols = cols_;
	
	//Initialise the maze size
	maze.resize(rows); 
	for(int i = 0; i < rows; i++){
		maze[i].resize(cols);
	}
	
	start = new AstarCell;
	goal = new AstarCell;
}

AstarSearch::~AstarSearch(){
	//dtor
	free(start);
	free(goal);
}

void AstarSearch::initialise(int startX, int startY, int goalX, int goalY){
	//Initialise Start vertex
	start->x = startX;
	start->y = startY; 
	start->g = 0;
	start->h = 0;
	
	//Initialise goal vertex
	goal->x = goalX; 
	goal->y = goalY; 
	goal->g = 0; 
	goal->h = 0;
	
	maze[start->y][start->x].g = start->g;
	maze[goal->y][goal->x].g = goal->g;
	
	cout << "Debug info dump: ==================================================" << endl;
	cout << "Position of start: X:" << start->x << " Y:" << start->y << endl;
	cout << "Positions of end: X:" << goal->x << " Y:" << goal->y << endl;
	cout << "initial distance from goal euclid: " << calculateH_euclid(start->x, start->y) << endl;
	cout << "initial distance from goal manhat: " << calculateH_manhat(start->x, start->y) << endl;
	cout << "====================================================================" << endl;
}

int AstarSearch::maxValue(int v1, int v2){
	if(v1 >= v2){
		return v1;
	} else {
		return v2;
	}	
}

//calculate the euclidean distance between a point and the goal
double AstarSearch::calculateH_euclid(int x, int y){
	int diff1 = abs(goal->y - y); 
	int diff2 = abs(goal->x - x);
	
	int p1 = pow(diff1, 2);
	int p2 = pow(diff2, 2);
	
	return (double)sqrt(p1 + p2); 
}

//calculate the manhatten distance between a point and the goal
double AstarSearch::calculateH_manhat(int x, int y){
	int diff1 = abs(goal->y - y);
	int diff2 = abs(goal->x - x);
	
	return (double)maxValue(diff1, diff2);
}
  
