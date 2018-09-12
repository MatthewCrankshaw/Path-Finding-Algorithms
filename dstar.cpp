#include "dstar.h"

dstar::dstar(int rows_, int cols_){
	rows = rows_; 
	cols = cols_; 
	
	maze.resize(rows);
	for(int i = 0; i < rows; i++){
		maze[i].resize(cols); 
	}
}

dstar::~dstar(){ 
	//dtor
}

void dstar::initialise(int startX, int startY, int goalX, int goalY){
	cout << "initialise dstar" << endl;
	U.clearQueue();
	km = 0;
}

void dstar::updateVertex(){
	cout << "update vertex dstar" << endl;
}

void dstar::computeShortestPath(){
	cout << "compute shortest path dstar" << endl;
}

void dstar::runDstar(){
	cout << "run dstar search" << endl;
}

