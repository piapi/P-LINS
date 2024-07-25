#include "dbscan.h"

int clusterID=0;
double select_MinPts(deque<point> &data, int k)
{
    vector<double> k_dist;
	// 使用openmp进行并行加速
// #pragma omp parallel for num_threads(2)
    for (int i = 0; i < data.size(); i++)
    {
        vector<double> temp;
        for (int j = 0; j < data.size(); j++)
        {
            double yu = sqrt((i-j) * (i-j) + (data[i].y - data[j].y) * (data[i].y - data[j].y));
            temp.push_back(yu);
        }
        sort(temp.begin(), temp.end());
        k_dist.push_back(temp[k]);
    }
    sort(k_dist.begin(), k_dist.end());
    for (int i = k_dist.size() - 1; i >= 3; i--)
    {
        if (k_dist[i] - k_dist[i - 1] < 0.1 && k_dist[i-1] - k_dist[i - 2] < 0.1 && k_dist[i-2] - k_dist[i - 3] < 0.1)
        {
            return k_dist[i];
        }
    }
    return 0;
}

void point::init(int idx)
{
    x=idx;
	cluster=0;
	pointType= pointType_UNDO ;//pointType_NOISE pointType_UNDO
	pts= 0 ;
	visited = 0;
	corePointID=-1;
}


float stringToFloat(string i){
	stringstream sf;
	float score=0;
	sf<<i;
	sf>>score;
	return score;
}



float squareDistance(point a,point b){
	return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

void DBSCAN(deque<point> &dataset,float Eps,int MinPts){
	int len = dataset.size();//数据长度
	for(int i=0;i<len;i++)//参数初始化
	{
		dataset[i].init(i);
	}

	deque<deque <float>> distP2P(len);

	//calculate pts
	//cout<<"calculate pts"<<endl;
	for(int i=0;i<len;i++){
		for(int j=0;j<len;j++){//i+1
			float distance= squareDistance(dataset[i],dataset[j]);
			distP2P[i].push_back(distance);//disp for debug
			if(distance<=Eps){
				dataset[i].pts++;
			}
		}
	}
	//core point 
	//cout<<"core point "<<endl;
	deque<point> corePoint;
	for(int i=0;i<len;i++){
		int tempPts=dataset[i].pts;
		if(tempPts>=MinPts) {
			dataset[i].pointType = pointType_CORE;
			dataset[i].corePointID=i;
			corePoint.push_back(dataset[i]);
		}
	}
	//cout<<"joint core point"<<endl;

	//joint core point
	int numCorePoint=corePoint.size(); //core point number

	for(int i=0;i<numCorePoint;i++){
		for(int j=0;j<numCorePoint;j++){
			float distTemp=distP2P[corePoint[i].corePointID][corePoint[j].corePointID];//display for debug
			if(distTemp<=Eps){//squareDistance(corePoint[i],corePoint[j])
				corePoint[i].corepts.push_back(j);//other point orderID link to core point
			}
		}
	}
	for(int i=0;i<numCorePoint;i++){
		stack<point*> ps;
		if(corePoint[i].visited == 1) continue;
		clusterID++;
		corePoint[i].cluster=clusterID; //create a new cluster
		ps.push(&corePoint[i]);
		point *v;
		while(!ps.empty()){
			v = ps.top();
			v->visited = 1;
			ps.pop();
			for(int j=0;j<v->corepts.size();j++){
				if(corePoint[v->corepts[j]].visited==1) continue;
				corePoint[v->corepts[j]].cluster = corePoint[i].cluster;
				corePoint[v->corepts[j]].visited = 1;
				ps.push(&corePoint[v->corepts[j]]);	
			}
		}	

	}

	//cout<<"border point,joint border point to core point"<<endl;
	//border point,joint border point to core point
	for(int i=0;i<len;i++){
		for(int j=0;j<numCorePoint;j++){
			float distTemp=distP2P[i][corePoint[j].corePointID];
			if(distTemp<=Eps) {
				dataset[i].pointType = pointType_BORDER;
				dataset[i].cluster = corePoint[j].cluster;
				break;
			}
		}
	}
}
