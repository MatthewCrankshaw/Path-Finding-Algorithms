#include <stdio.h>
#include <vector>
#include <malloc.h>
#include <assert.h>
#include <algorithm>    
#include <math.h>
#include <climits>

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



bool AstarSearch::computeShortestPath(int &numOfVertexExpansions, int &maxQlen, int &vertexAccesses){
	// First put the starting node into the open list
	start->h = calculateH_euclid(start->x, start->y);
	start->g = 0;
	open.push_back(*start);
	cout << "position of start node is: " << open[0].x << " " << open[0].y << " h: " << open[0].h << endl;
	
	while(!open.empty()){
		AstarCell node_current;
		node_current = lowestFCostInOpen();
		
		//if the current node is the goal node then you have found the goal
		if(isGoal(node_current)){
			cout << "Goal Found!" << endl;
			break;
		}
		//generate the successor nodes
		generateChildNodes(node_current);
		for(int i = 0; i < DIRECTIONS; i++){
			//is in open list
			int succG;
			//returns null if not found
			succG = inOpenArray(node_current.move[i]);
			cout << "in open array " << succG << endl;
			if(succG != -1){
				if(succG <= node_current.move[i]->g) continue;
			}//else if(){ //is in the closed list
				
			//}else{ //is not in any list
				
			//}
		}
		
		
		//remove
		break;
	}
	
	
	
	return false;
}

void AstarSearch::printMaze(){
	for(int i = 0; i < cols; i ++){
		for(int j = 0; j < rows; j++){
			cout << (char)maze[j][i].type << " ";
		}
		cout << endl;
	}
}

void AstarSearch::generateChildNodes(AstarCell &node){
	//TYPE: 0 - traversable, 1 - blocked, 9 - unknown, 6 - start vertex, 7 - goal vertex
	
	int x[8] = {
		node.x - 1, node.x, node.x + 1,
		node.x - 1, node.x + 1, 
		node.x - 1, node.x, node.x + 1
	}; 

	int y[8] = {
		node.y - 1, node.y - 1, node.y - 1,
		node.y, node.y,
		node.y + 1, node.y + 1, node.y + 1
	};
	
	double diag = sqrt(2);
	double cost[8] = {
		diag, 1, diag,
		1      , 1,
		diag, 1, diag
	};
	
	for(int i = 0; i < DIRECTIONS; i++){ 
		
		node.move[i] = new AstarCell;
		//if the node is off the grid set not traversable
		if(x[i] < 0 || x[i] > rows || y[i] < 0 || y[i] > cols){
			node.move[i]->type = 1;
			continue;
		}
		
		node.move[i]->x = x[i];
		node.move[i]->y = y[i];
		node.move[i]->type = maze[y[i]][x[i]].type;
		node.move[i]->g = node.g + cost[i];
		node.move[i]->h = calculateH_euclid(x[i], y[i]);
		cout << "next type " << (char)node.move[i]->type << " x " << x[i] << " y " << y[i] << " cost " << cost[i] << " h " << node.move[i]->h << endl;
	}
}

//returns -1 if not in open array or returns the g cost
int AstarSearch::inOpenArray(const AstarCell *a){
	for(auto node : open){
		if((a->x == node.x) && (a->y == node.y)){ 
			return node.g;
		}
	}
	return -1;
}

bool AstarSearch::isGoal(const AstarCell a){
	if((a.x == goal->x) &&(a.y == goal->y)){
		return true;
	}
	return false;
}

AstarCell AstarSearch::lowestFCostInOpen(){
	AstarCell lowest;
	lowest.g = 10000.00;
	lowest.h = 10000.00;
	
	for(auto node : open){
		if(calculateFCost(node) < calculateFCost(lowest)){
			//cout << "lowest: " << node.x << " " << node.y << endl;
			lowest = node;
		}
	}
	
	return lowest;
}

double AstarSearch::calculateFCost(const AstarCell a){
	return a.g + a.h;
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
  
