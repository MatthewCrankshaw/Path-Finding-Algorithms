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
}
  
