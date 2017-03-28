#include "Graph.h"

Graph::Graph(int dimX, int dimY): map(dimY, vector<Node>(dimX)), openNodes(), closedNodes() {
}

Node* Graph::getCellValue(pair<int, int> coord){
	return &(this->map[coord.first][coord.second]);
}

void Graph::setCellValue(pair<int, int> coord, char value){
	this->map[coord.first][coord.second].setState(Node::UNVISITED);
	this->map[coord.first][coord.second].setType(value);
	this->map[coord.first][coord.second].x = coord.first;
	this->map[coord.first][coord.second].y = coord.second;
}

pair<int, int> Graph::getRobotPos(int n){
	if (n==1)
		return this->firstRobotPos;
	else
		return this->secondRobotPos;
}

pair<int, int> Graph::getTargetPos(){
	return this->targetPos;
}

void Graph::setTargetPos(pair<int, int> coord){
	this->targetPos = coord;
};

void Graph::setFirstRobotPos(pair<int, int> coord){
	this->firstRobotPos = coord;
}

void Graph::setSecondRobotPos(pair<int, int> coord){
	this->secondRobotPos = coord;
}

/* Caluclate Manhattan Distance of 2 points */
float Graph::calculateDistance(pair<int, int> from, pair<int, int> to){
	return abs( ( double )from.first - to.first ) + abs( ( double )from.second - to.second );
}

bool Graph::compareNodes::operator()(Node n1, Node n2){
		return n1.getTotalCost() > n2.getTotalCost();
}

/* Method to check if a Point is in the map */
bool Graph::isInsideMap(pair<int, int> coord){
	return (coord.first >= 0) && (coord.second >= 0) && (coord.first < map[0].size()) && (coord.second < map.size()) ;
}

/* Method to find all valid neighbour Nodes of a given node */
set<Node> Graph::getNeighbours(Node n){
	set<Node> neighbours;
	Node* temp;
	list<int> values = {-1, 0, 1};

	for(int i : values){
		for(int j : values){
			if ((i== 0 && j!=0) || (j==0 && i!=0)){
				if( (isInsideMap(make_pair(n.x+i, n.y+j))) ){
					temp = getCellValue(make_pair((n.x+i), (n.y+j)));
					//printf("found neighbour:(%d,%d)\n",make_pair(n.x+i,n.y+j).second,make_pair(n.x+i,n.y+j).first);
					if( (!temp->isObstacle())){
						neighbours.insert(*temp);
					}
				}
			}
		}
	}
	return neighbours;
}

/* Method to reconstruct Path from source to target Node*/
vector<Node> Graph::reconstructPath(Node* to, Node* from){
	vector<Node> path;
	Node* tmp = from;
	Node* parent;

	path.push_back(*tmp);
	while(tmp->getParent() != nullptr){
		parent = (tmp->getParent());
		path.push_back(*parent);
		tmp = parent;
	}
	reverse(path.begin(), path.end());
	return path;
}

/* Method to print a path of Nodes */
void Graph::printPath(vector<Node> somePath){
	cout << "--- Path to target ---" << endl;
	for(auto i=somePath.begin(); i != somePath.end(); i++){
		Node node = *i;
		cout << "node : (" << node.y << "," << node.x << ")" << endl;
	}
}

/* Method that implements main A-Star Algorithm */
vector<Node> Graph::executeAStar(pair<int, int> StartPoint , pair<int, int> EndPoint){
	pair <int, int> source = make_pair(StartPoint.first, StartPoint.second);
	pair <int, int> target = make_pair(EndPoint.first, EndPoint.second);
	Node* startNodePtr = getCellValue(source);
	Node* targetNodePtr = getCellValue(target);
	vector<Node> path;
	Node currentNode;
	set<Node> neighbours;
	openNodes = priority_queue <Node , vector<Node>, compareNodes>();
	closedNodes = priority_queue <Node , vector<Node>, compareNodes>();
	openNodes.push(*startNodePtr);
	startNodePtr->setOpen();

	while(!openNodes.empty()){
		currentNode = openNodes.top();
		Node* currentPtr = getCellValue(make_pair(currentNode.x, currentNode.y));
		openNodes.pop();
		closedNodes.push(*currentPtr);
		currentPtr->setClosed();
		neighbours = getNeighbours(*currentPtr);
		if( (currentPtr->x == targetNodePtr->x) && (currentPtr->y == targetNodePtr->y) ){
			return reconstructPath(startNodePtr, currentPtr);
		}
		for(auto i = neighbours.begin(); i != neighbours.end(); ++i){
			Node* neighbourPtr = getCellValue(make_pair(i->x, i->y));
			if(!(neighbourPtr->isClosed())){
				int tentativeScore = currentNode.getCostFromStart() + this->calculateDistance(make_pair(currentPtr->x, currentPtr->y), make_pair(neighbourPtr->x, neighbourPtr->y));
				if( (!neighbourPtr->isOpen()) || (tentativeScore < currentNode.getCostFromStart()) ){
					neighbourPtr->setParent(currentPtr);
					neighbourPtr->setCostFromStart(tentativeScore);
					neighbourPtr->setCostToTarget(this->calculateDistance(make_pair(neighbourPtr->x, neighbourPtr->y), target));
					neighbourPtr->calculateTotalCost();
					if(!neighbourPtr->isOpen()){
						openNodes.push(*neighbourPtr);
						neighbourPtr->setOpen();
					}
				}
			}
		}
	}
	return path;
}
