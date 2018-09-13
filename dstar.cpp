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

void dstar::updateVertex(){
	cout << "update vertex dstar" << endl;
}

void dstar::computeShortestPath(){
	cout << "compute shortest path dstar" << endl;
}

void dstar::runDstar(){
	cout << "run dstar search" << endl;
	/*for(int i = 0; i < rows; i++){
		for(int j = 0; j < cols; j++){
			cout << "[" << maze[i][j].type << " {" << maze[i][j].key[0] << " " << maze[i][j].key[1] << "} " << maze[i][j].g << " " << maze[i][j].rhs << "]";
		}
		cout << endl;
	}*/
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
	
	key2 = minValue(node->g, node->rhs);
	key1 = key2 + node->h;
	
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

