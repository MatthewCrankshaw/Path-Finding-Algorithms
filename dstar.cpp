#include <stdlib.h>     /* calloc, exit, free */
#include <math.h>  //sqrt, pow
#include<iostream>

#include "dstar.h"
#include "gridworld.h"

dstar::dstar(int rows_, int cols_){
	rows = rows_; 
	cols = cols_; 
	cout << rows << " " << cols << endl;
	maze.resize(rows);
	for(int i = 0; i < rows; i++){
		maze[i].resize(cols); 
	}
}

dstar::~dstar(){ 
	//dtor
	free(start);
	free(goal);
}

void dstar::initialise(int startX, int startY, int goalX, int goalY){
	cout << "initialise dstar" << endl;
	
	start = new dStarNode();
	goal = new dStarNode();
	
	//Set up start and goal node
	start->x = startX; 
	start->y = startY; 
	start->rhs = INF; 
	start->g = INF;
	
	goal->x = goalX; 
	goal->y = goalY;
	goal->rhs = 0;
	goal->g = INF;
	
	// clear the priority queue
	U.clearQueue();
	//initialise km
	km = 0;
	
	//set all of the g and rhs costs to INF
	for(int i = 0; i < rows; i++){
		for(int j = 0; j < cols; j++){ 
			maze[i][j].g = INF; 
			maze[i][j].rhs = INF;
		}
	}
	
	//update all of the H values
	updateHValues();
	//Calculate the key of the goal
	calcKey(goal);
	maze[goal->y][goal->x].key[0] = goal->key[0];
	maze[goal->y][goal->x].key[1] = goal->key[1];
	
	//insert the goal node into the priority queue
	U.insert(*goal);
}

void dstar::runDstar(int startX, int startY, int goalX, int goalY){
	cout << "run dstar search" << endl;
	initialise(startX, startY, goalX, goalY);
	computeShortestPath();
}

void dstar::computeShortestPath(){
	cout << "compute shortest path dstar" << endl;
	
	double a[2];
	
	U.topkey(a);
	
	while(shouldLoop()){
		//set kold to be the smallest k
		double kold[2];
		U.topkey(kold);
		//pull the smallest k node off the priority queue
		dStarNode u = U.pop();
		calcKey(&u);
		
		if(smallerKey(kold, u.key)){
			U.insert(u);
		}else if(u.g > u.rhs){ 
			u.g = u.rhs;
			maze[u.y][u.x] = u;
			for(int i = 0; i < DIRECTIONS; i++){
				int x[8] = {u.x-1, u.x, u.x+1, u.x-1, u.x+1, u.x-1, u.x, u.x+1};
				int y[8] = {u.y-1, u.y-1, u.y-1, u.y, u.y, u.y+1, u.y+1, u.y+1};
				
				if(x[i] < 0 || y[i] < 0 || x[i] > cols || y[i] > rows){
					continue;
				}
				if(maze[y[i]][x[i]].type == '1') continue;
				
				dStarNode s = maze[y[i]][x[i]];
				updateVertex(s);
			}
		}else{
			u.g = INF;
			for(int i = 0; i < DIRECTIONS; i++){
				int x[9] = {u.x-1, u.x, u.x+1, u.x-1, u.x ,u.x+1, u.x-1, u.x, u.x+1};
				int y[9] = {u.y-1, u.y-1, u.y-1, u.y, u.y, u.y, u.y+1, u.y+1, u.y+1};
				
				if(x[i] < 0 || y[i] < 0 || x[i] > cols || y[i] > rows){
					continue;
				}
				if(maze[y[i]][x[i]].type == '1') continue;
				
				dStarNode s = maze[y[i]][x[i]];
				updateVertex(s);
			}
		}
		U.printqueue();
		cout << endl;
		for(int i = 0; i < rows; i++){
			for(int j = 0; j < cols; j++){
				cout << "[" << maze[i][j].type << " {" << maze[i][j].key[0] << " " << maze[i][j].key[1] << "} " << maze[i][j].g << " " << maze[i][j].rhs << "]";
			}
			cout << endl;
		}	
		cout << endl;
		break;
	}
	
}

void dstar::updateVertex(dStarNode s){
	cout << "update vertex dstar" << endl;
	
	if(s.x != goal->x && s.y != goal->y){ 
		cout << "updating not goal" << endl;
		double min = INF;
		for(int i = 0; i < DIRECTIONS; i++){
			int x[8] = {s.x-1, s.x, s.x+1, s.x-1 ,s.x+1, s.x-1, s.x, s.x+1};
			int y[8] = {s.y-1, s.y-1, s.y-1, s.y, s.y, s.y+1, s.y+1, s.y+1};
			
			if(x[i] < 0 || y[i] < 0 || x[i] > cols || y[i] > rows){
				continue;
			}
			
			if(maze[y[i]][x[i]].type == '1') continue;
			cout << x[i] << " " << y[i] << endl;
				
			cout << "cost s: " << s.g << " cost of succ " << maze[y[i]][x[i]].g << endl;
			
			double c = s.g + maze[y[i]][x[i]].g;
			if(c < min){
				min = c;
			}
		}
		if(min == INF) min = 1;
		cout << "min " << min << endl;
		s.rhs = min;
		maze[s.y][s.x] = s;
	}
	if(U.exists(s.x,s.y)){
		cout << "s exists in priority queue - update " << endl;
		U.remove(s);
	}
	if(s.g != s.rhs){
		cout << "s g does not = s rhs ... insert to queue - update " << endl;
		calcKey(&s);
		maze[s.y][s.x] = s;
		U.insert(s);
	}
}

//if a key is smaller than b key returns true
//otherwise false
bool dstar::smallerKey(double *a, double*b){
	if(a[0] < b[0]){
		return true;
	}else{ 
		if((a[0] == b[0]) && (a[1] < b[1])){
			return true; 
		}else{
			return false;
		}
	}
}

bool dstar::shouldLoop(){
	double top[2]; 
	
	U.topkey(top);
	calcKey(start);
	
	if(smallerKey(top, start->key)){
		return true;
	}else{
		if(start->rhs != start->g) return true;
		else return false;
	}
}

double dstar::minValue(double g_, double rhs_){
	if(g_ <= rhs_){
		return g_;
	} else {
		return rhs_;
	}	
}

int dstar::maxValue(int v1, int v2){
	if(v1 >= v2){
		return v1;
	} else {
		return v2;
	}	
}

void dstar::calcKey(dStarNode *node){
	double key1, key2;
	
	key1 = minValue(node->g, node->rhs) + node->h + km;
	key2 = minValue(node->g, node->rhs);
	
	node->key[0] = key1;
	node->key[1] = key2;
}

double dstar::calc_H(int x, int y){
	
	int diffY = abs(start->y - y);
	int diffX = abs(start->x - x);
	
	//maze[y][x].h = (double)maxValue(diffY, diffX);
	return (double)maxValue(diffY, diffX);
}

void dstar::updateHValues(){
	for(int i=0; i < rows; i++){
	   for(int j=0; j < cols; j++){
		   maze[i][j].h = calc_H(j, i);
		}
	}
	
	start->h = calc_H(start->x, start->y);
	goal->h = calc_H(goal->x, goal->y);
}

