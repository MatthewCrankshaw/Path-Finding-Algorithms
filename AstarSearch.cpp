#include <stdio.h>
#include <vector>
#include <malloc.h>
#include <assert.h>
#include <algorithm>    
#include <math.h>
#include <cstring>

#include "AstarSearch.h"
#include "globalVariables.h"

using namespace std;

AstarSearch::AstarSearch(int rows_, int cols_){
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
	free(start);
	free(goal);
	for(auto i : closed){
		free(i.parent);
	}
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
	
	maze[start->x][start->y].g = start->g;
	maze[goal->x][goal->y].g = goal->g;
}



vector<AstarCell> AstarSearch::computeShortestPath(int &numOfVertexExpansions, int &maxQlen, int &vertexAccesses){
	// First put the starting node into the open list
	
	maxQlen = 0;
	
	//change to switch between manhattan distance calculation and Euclidean
	if(DISTANCE_CALC == 1){ 
		start->h = calculateH_euclid(start->x, start->y);
	}else{
		start->h = calculateH_euclid(start->x, start->y);
	}
	
	start->g = 0;
	start->parent = NULL;
	open.push_back(*start);
	
	AstarCell node_current;
	while(!open.empty()){
		node_current = lowestFCostInOpen();
		
		//if the current node is the goal node then you have found the goal
		if(isGoal(node_current)){
			cout << "Goal Found!" << endl;
			AstarCell *p; 
			p = &node_current; 
			
			//forever
			for(;"ever";){
				if(p == NULL){
					break;
				}
				final_path.push_back(*p);
				p = p->parent;
			}
			break;
		}
		//generate the successor nodes
		generateChildNodes(node_current);
		
		for(int i = 0; i < DIRECTIONS; i++){
			double currCost = node_current.g + cost[i];
			AstarCell *next = node_current.move[i];
			//is in the lists
			int openG, closedG;
			if(next->type != '1'){
				
				//returns null if not found
				openG = inOpenArray(next);
				closedG = inClosedArray(next);
				
				if(openG != -1){
					if(openG <= currCost) continue;
				}else if(closedG != -1){ //is in the closed list
					if(closedG <= currCost) continue;
				}else{ //is not in any list
					open.push_back(*next);
					if(open.size() > maxQlen){ 
						maxQlen = open.size();
					}
				}
			}
		}
		//printNode(&node_current);
		closed.push_back(node_current);
	}
	if(!isGoal(node_current)){
		cout << "goal not found: open list is empty" << endl;
		exit(1);
	}
	
	for(auto i : closed){
		maze[i.y][i.x].g = i.g; 
		maze[i.y][i.x].h = i.h;
	}
	numOfVertexExpansions = closed.size();
	
	cout << "Path is: " << endl;
	for(auto i : final_path){ 
		cout << "X: " << i.x << " Y: " << i.y << endl;
	}
	
	return final_path;
}

void AstarSearch::printNode(AstarCell *a){
	cout << "x " << a->x  << " y " << a->y << " g " << a->g << " h " << a->h << " type " << a->type << endl;
}

void AstarSearch::printMaze(){
	for(int i = 0; i < rows; i ++){
		for(int j = 0; j < cols; j++){
			cout << (char)maze[i][j].type << " ";
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
		if(x[i] < 0 || x[i] > cols || y[i] < 0 || y[i] > rows){
			node.move[i]->type = '1';
			continue;
		}
		
		node.move[i]->x = x[i];
		node.move[i]->y = y[i];
		node.move[i]->type = maze[y[i]][x[i]].type;
		node.move[i]->g = cost[i];// + node.g;
		if(DISTANCE_CALC == 1){ 
			node.move[i]->h = calculateH_euclid(x[i], y[i]);;
		}else{
			node.move[i]->h = calculateH_manhat(x[i], y[i]);
		}
		node.move[i]->parent = new AstarCell;
		node.move[i]->parent->x = node.x; 
		node.move[i]->parent->y = node.y; 
		node.move[i]->parent->parent = node.parent;
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

//returns -1 if not in the closed array or returns the g cost
int AstarSearch::inClosedArray(const AstarCell *a){
	for(auto node : closed){
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
	vector<AstarCell>::iterator lowItr;
	
	lowest.g = 10000.00;
	lowest.h = 10000.00;
	for(vector<AstarCell>::iterator itr = open.begin(); itr != open.end(); itr++){
		//printNode(&(*itr));
		if(calculateFCost(*itr) < calculateFCost(lowest)){
			lowest = *itr;
			lowItr = itr;
		}
	}
	open.erase(lowItr);
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
  
