#include "pqueue.h"


pqueue::pqueue(){
	//ctor
}

pqueue::~pqueue(){
	//dtor
}

void pqueue::remove(dStarNode v){
	for(vector<dStarNode>::iterator iter = vect.begin(); iter != vect.end(); iter++){
		int x = iter->x;
		int y = iter->y;

		if(x == v.x && y == v.y){
			vect.erase(iter);
			return;
		}
	}

	cout << "Error could not find dStarNode : " << v.x << " " << v.y << " in priority queue to remove" << endl;
	exit(1);
}

void pqueue::insert(dStarNode v){ 
	//inserts dStarNode v into priority queue with priority key
	vect.push_back(v);
}

void pqueue::update(dStarNode v){ 
	//updates existing dStarNode v with new priority key
	for(vector<dStarNode>::iterator iter = vect.begin(); iter != vect.end(); iter++){
		int x = iter->x;
		int y = iter->y;

		if(x == v.x && y == v.y){
			iter->key[0] = v.key[0];
			iter->key[1] = v.key[1];
			return;
		}
	}
	cout << "Unable to find and update dStarNode in priority queue: " << v.x << " " << v.y << endl;
	exit(1);
}

dStarNode pqueue::pop(){ 
	//removes dStarNode with smallest priority and returns the dStarNode
	vector<dStarNode>::iterator iterDel;
	dStarNode ret;

	iterDel = vect.begin();
	ret = *vect.begin();

	for(vector<dStarNode>::iterator iter = vect.begin(); iter != vect.end(); iter++){
		if(comparepriorities(*iterDel, *iter) == 1){
			iterDel = iter;
			ret = *iter;
		}
	}

	vect.erase(iterDel);
	return ret;

}

void pqueue::topkey(double *key){ 
	//returns the dStarNode with the smallest priority
	//if empty then returns [INF, INF]

	//vector is empty
	if(vect.empty()){
		key[0] = numeric_limits<double>::infinity();
		key[1] = numeric_limits<double>::infinity();
		return;
	}

	//go through each item and find smallest key
	dStarNode ret = *vect.begin(); 
	for(auto i : vect){
		if(comparepriorities(i, ret) == -1){
			ret = i;
		}
	}
	//return the smallest key
	key[0] = ret.key[0]; 
	key[1] = ret.key[1];
}

dStarNode pqueue::top(){
	if(vect.empty()){
		cout << "Error: vect empty!!" << endl;
		exit(1);
	}
	dStarNode ret = *vect.begin(); 
	for(auto i : vect){
		if(comparepriorities(i, ret) == -1){
			ret = i;
		}
	}
	
	return ret;
}

bool pqueue::exists(int x, int y){
	for(auto i : vect){ 
		if(i.x == x && i.y == y){
			return true;
		}
	}
	return false;
}

unsigned pqueue::size(){ 
	return vect.size();
}

void pqueue::clearQueue(){
	vect.clear();
}


void pqueue::printqueue(){ 
	for(auto i : vect){
		cout << i.x << " " << i.y << " " << i.rhs<< " "  << i.g<< " {"  << i.key[0] << " " << i.key[1]<< "} "  << endl;
	}
}

//compare 2 priorities
//-1 for a smaller key 
// 1 for b smaller key
int pqueue::comparepriorities(dStarNode a, dStarNode b){ 
	if(a.key[0] == b.key[0]){
		if(a.key[1] < b.key[1]){
			return -1;
		}else if(a.key[1] > b.key[1]){
			return 1;
		}
	}else if(a.key[0] < b.key[0]){ 
		return -1;
	}else if(a.key[0] > b.key[0]){
		return 1;
	}
	return -1;
}