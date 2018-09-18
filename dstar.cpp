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
	maxQLen = 0;
	numExpan = 0;
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
	start->type = '6';
	
	goal->x = goalX; 
	goal->y = goalY;
	goal->rhs = 0;
	goal->g = INF;
	goal->type = '7';
	
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
	initialise(startX, startY, goalX, goalY);
	computeShortestPath();
}

void dstar::computeShortestPath(){
	while(shouldLoop()){
		//set kold to be the smallest k
		double kold[2], knew[2];
		dStarNode u, old;

		u = U.top();
		old = U.top();
		
		U.topkey(kold);
		
		calcKey(&u);
		knew[0] = u.key[0];
		knew[1] = u.key[1];
		
		if(smallerKey(kold, knew)){
			cout << "kold smaller than knew" << endl;
			u.key[0] = knew[0]; 
			u.key[1] = knew[1];
			U.update(u);
			maze[u.y][u.x] = u;
		}else if(u.g > u.rhs){
			cout << "g is greater than rhs" << endl;
			cout << "x " << u.x << " y " << u.y << " g: " << u.g << " rhs " << u.rhs << endl;
			u.g = u.rhs;
			U.remove(u);
			maze[u.y][u.x] = u;
			
			for(int i = 0; i < DIRECTIONS; i++){
				int x[8] = {u.x-1, u.x, u.x+1, u.x-1, u.x+1, u.x-1, u.x, u.x+1};
				int y[8] = {u.y-1, u.y-1, u.y-1, u.y, u.y, u.y+1, u.y+1, u.y+1};
				
				if(x[i] < 0 || y[i] < 0 || x[i] > cols || y[i] > rows){
					continue;
				}
				if(maze[y[i]][x[i]].type == '1') continue;
				
				dStarNode s = maze[y[i]][x[i]];
				cout << "x " << x[i] << " y  " << y[i] << endl; 
				cout << "not goal " << s.x << " " << s.y << endl;
				if(s.x != goal->x || s.y != goal->y){
					s.rhs = minValue(s.rhs, cost[i] + u.g);
					maze[s.y][s.x] = s;
				}
				updateVertex(s);
			}
		}else{
			cout << "else" << endl;
			old.g = u.g;
			maze[old.y][old.x] = old;
			u.g = INF;
			maze[u.y][u.x] = u;
			
			
			for(int i = 0; i < DIRECTIONS; i++){
				int x[9] = {u.x-1, u.x, u.x+1, u.x-1, u.x ,u.x+1, u.x-1, u.x, u.x+1};
				int y[9] = {u.y-1, u.y-1, u.y-1, u.y, u.y, u.y, u.y+1, u.y+1, u.y+1};
				
				if(x[i] < 0 || y[i] < 0 || x[i] > cols || y[i] > rows){
					continue;
				}
				if(maze[y[i]][x[i]].type == '1') continue;
				
				dStarNode s = maze[y[i]][x[i]];
				if(s.rhs == cost[i] + old.g){
					if(s.x != goal->x || s.y != goal->y){
						double min = INF;
						for(int j = 0; j < DIRECTIONS; j++){
							int x[8] = {s.x-1, s.x, s.x+1, s.x-1, s.x+1, s.x-1, s.x, s.x+1};
							int y[8] = {s.y-1, s.y-1, s.y-1, s.y, s.y, s.y+1, s.y+1, s.y+1};
								
							if(x[i] < 0 || y[i] < 0 || x[i] > cols || y[i] > rows){
								continue;
							}
							if(maze[y[i]][x[i]].type == '1') continue;
								
							if(maze[y[i]][x[i]].g + cost[i] < min){
								min = maze[y[i]][x[i]].g + cost[i];
							}
						}
						s.rhs = min;
						cout << "min " << min;
						maze[y[i]][x[i]] = s;
					}
				}
				updateVertex(s);
				maze[s.y][s.x] = s;
			}
		}
		cout << "in while loop " << endl;
	}
	cout << "compute shortest path exit " << endl;
}

void dstar::updateVertex(dStarNode s){
	if((s.g != s.rhs) && (U.exists(s.x, s.y))){
		calcKey(&s);
		U.update(s);
		maze[s.y][s.x] = s;
	}else if((s.g != s.rhs) && (!U.exists(s.x, s.y))){
		calcKey(&s);
		U.insert(s);
		if(maxQLen < U.size()) maxQLen += 1;
		maze[s.y][s.x] = s;
	}else if((s.g == s.rhs) && (U.exists(s.x, s.y))){
		U.remove(s);
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
		if(start->rhs > start->g) return true;
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

double dstar::calc_H_euclid(int x, int y){
	
	int diffY = abs(start->y - y);
	int diffX = abs(start->x - x);
	
	int p1 = pow(diffY, 2);
	int p2 = pow(diffX, 2);
	
	return (double)sqrt(p1 + p2);
}

double dstar::calc_H_manhat(int x, int y){
	int diffY = abs(start->y - y);
	int diffX = abs(start->x - x);
	return (double)maxValue(diffY, diffX);
}

void dstar::updateHValues(){
	if(DISTANCE_CALC == 1){
		for(int i=0; i < rows; i++){
			for(int j=0; j < cols; j++){
				maze[i][j].h = calc_H_euclid(j, i);
			}
		}
		
		start->h = calc_H_euclid(start->x, start->y);
		goal->h = calc_H_euclid(goal->x, goal->y);
	}else{
		for(int i=0; i < rows; i++){
			for(int j=0; j < cols; j++){
				maze[i][j].h = calc_H_manhat(j, i);
			}
		}
		
		start->h = calc_H_manhat(start->x, start->y);
		goal->h = calc_H_manhat(goal->x, goal->y);
	}
}

unsigned dstar::getMaxQLen(){ 
	return maxQLen;
}

