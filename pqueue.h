#include<iostream>
#include<vector>
#include<limits>
#include<cstdlib>
#include "globalVariables.h"

using namespace std; 

class pqueue{
public:
	pqueue();
	~pqueue();

	void insert(dStarNode v);
	void update(dStarNode v);
	void remove(dStarNode v);
	bool exists(int x, int y);
	dStarNode pop(); 
	void topkey(double *key); 
	void printqueue();
	int comparepriorities(dStarNode a, dStarNode b);
	void clearQueue();

private: 

	vector<dStarNode> vect;

};