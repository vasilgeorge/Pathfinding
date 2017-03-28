#include <iostream>
#include <assert.h>
#include <vector>
#include <fstream>
#include <stdexcept>
#include <string>
#include <algorithm>

#include "Graph.h"

using namespace std;

int main(int argc, char* argv[]){
	if(argc != 2){
		cout << "Usage:robots " << "<filename>" << endl;
		return 1;
	}
	else{
			string filename = argv[1];
      int x,y,CP;
			int mapD,cols,rows;
      vector <pair <int, int> > Checkpoints;
			vector <Node> totalPathA, totalPathB;
      string line;
      char type;
			pair<int, int> src;
			pair<int, int> dst;
			ifstream inputFile(filename.c_str());
			if (inputFile){
				try{
          inputFile >> cols;
          inputFile >> rows;
					mapD = max(cols,rows);

					/* Create Map Object */
          Graph graph(mapD, mapD);

					getline(inputFile, line);
          inputFile >> y;
          inputFile >> x;
          graph.setFirstRobotPos(make_pair(x,y));

					getline(inputFile, line);
          inputFile >> y;
          inputFile >> x;
          graph.setSecondRobotPos(make_pair(x,y));

					getline(inputFile, line);
          inputFile >> y;
          inputFile >> x;
					pair<int, int> target = make_pair(x,y);
          graph.setTargetPos(target);

					getline(inputFile, line);
          inputFile >> CP;
          getline(inputFile, line);

          for(int i=0; i<CP; i++){
            inputFile >> y;
            inputFile >> x;
            Checkpoints.push_back(make_pair(x,y));
          }

					/* Add targetPos as last checkpoint */
					Checkpoints.push_back(target);

					if (CP > 0)
						getline(inputFile, line);

          for(int i = 0; i<rows; i++){
							line.clear();
							getline(inputFile, line);
							cout << line << endl;
              for(int j = 0; j<cols; j++){
                assert(line.at(j) == 'X' || line.at(j) == 'O');
								type = line.at(j);
                graph.setCellValue(make_pair(i, j), type);
              }
              line.clear();
          }

					/* Execute A-Star Algorithm for robot1 */
					src = graph.getRobotPos(1);
					for(vector<pair<int, int>>::iterator it=Checkpoints.begin(); it!=Checkpoints.end(); ++it){
						dst = make_pair(it->first,it->second);
						vector <Node> path = graph.executeAStar(src, dst);
						totalPathA.insert(totalPathA.end(), path.begin(), path.end());
						src = dst;
					}
					cout << "Total number of robot A moves: " << totalPathA.size() << endl;;
          cout << "Whole pathA to the target: " << endl;
          graph.printPath(totalPathA);

					/* Execute A-Star Algorithm for robot2 */
					src = graph.getRobotPos(2);
					for(vector<pair<int, int>>::iterator it=Checkpoints.begin(); it!=Checkpoints.end(); ++it){
						dst = make_pair(it->first,it->second);
						vector <Node> path = graph.executeAStar(src, dst);
						totalPathB.insert(totalPathB.end(), path.begin(), path.end());
						src = dst;
					}
					cout << "Total number of robot B moves: " << totalPathB.size() << endl;;
					cout << "Whole pathB to the target: " << endl;
					graph.printPath(totalPathB);

					inputFile.close();
				}
				catch(exception &e){
					inputFile.close();
					throw runtime_error("Input file is not well formatted.");
			  }
		  }
      else
        printf("Could not open input file.\n");
   return 0;
   }
}
