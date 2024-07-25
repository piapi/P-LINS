#if !defined(_DBSCAN_H_)
#define _DBSCAN_H_

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <cmath>
#include <stack>
#include <map>
#include <algorithm>
#include <math.h>
using namespace std;

enum
{
	pointType_UNDO,
	pointType_NOISE,
	pointType_BORDER,
	pointType_CORE

};

class point{
public:
	float x;
	float y;
	int cluster;
	int pointType;  //1 noise 2 border 3 core
	int pts;        //points in MinPts 
	int corePointID;
	deque<int> corepts;
	int  visited;
	void init(int idx);
	point () ;
	point (float val){
		y = val;
	};
};


float stringToFloat(string i);
float squareDistance(point a,point b);
void DBSCAN(deque<point> &dataset,float Eps,int MinPts);
double select_MinPts(deque<point> &data, int k);
#endif 
