#ifndef Graph_H_
#define Graph_H_

#include <vector>
#include <algorithm>
#include <set>
#include <list>
#include <queue>
#include <math.h>
#include <iostream>
#include "Node.h"
using namespace std;

class Graph {
private:
	vector< vector< Node > > map;
	pair<int, int> firstRobotPos;
	pair<int, int> secondRobotPos;
	pair<int, int> targetPos;
public:
	Graph(int mapDimensionX, int mapDimensionY);
	Node* getCellValue(pair<int, int> coord);
	void setCellValue(pair<int, int> coord, char value);
	void setFirstRobotPos(pair<int, int> coord);
	void setSecondRobotPos(pair<int, int> coord);
	void setTargetPos(pair<int, int> coord);
	pair<int, int> getRobotPos(int n);
	pair<int, int> getTargetPos();
	float calculateDistance(pair<int, int> from, pair<int, int> to);
	bool isInsideMap(pair<int, int> coord);
	class compareNodes{
		public:
		bool operator()(Node n1, Node n2);
	};
	set<Node> getNeighbours(Node n);
	vector<Node> executeAStar(pair <int, int>, pair<int, int>);
	vector<Node> reconstructPath(Node* to, Node* from);
	void printPath(vector<Node> path);
	priority_queue<Node , vector<Node>, compareNodes> openNodes;
	priority_queue<Node, vector<Node>, compareNodes> closedNodes;
};

#endif /* Graph_H_ */
