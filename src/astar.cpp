//
// Created by Vivek Shankar on 12/6/21.
//
#define ROW 20
#define COLUMN 18
#define MAP_SIZE_X 18
#define MAP_SIZE_Y 19.6

#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <limits>
#include <set>

#define GetCurrentDir getcwd

using namespace std;

struct node {

    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
};


class Astar {

private:
    ros::NodeHandle n;

    int map[20][18];
    pair<double, double> start_xy = make_pair(-8.0, -2.0);

    node node[ROW][COLUMN];

    // Meant to store row, col of the cell where the src or goal lies.
    pair<int, int> src;
    pair<int, int> goal;

    pair<int, int> origin = make_pair(ROW / 2, COLUMN / 2);


public:
    Astar() {
        n = ros::NodeHandle("~");

        double goal_x, goal_y;
        n.getParam("/goalx", goal_x);
        n.getParam("/goaly", goal_y);
        cout << "For goal at: (" << goal_x << ", " << goal_y << ")" << endl;

        cout << "Origin: (" << origin.first << " ," << origin.second << ")";

        // Use co-ordinates and size of map to evaluate the i, j of src and goal;
        src.first = (int) (origin.first - start_xy.second);
        src.second = (int) (origin.second + start_xy.first);

        cout << "Src_ij: (" << src.first << " ," << src.second << ")";

        goal.first = (int) (origin.first - goal_y);
        goal.second = (int) (origin.second + goal_x);

        cout << "Goal_ij: (" << goal.first << " ," << goal.second << ")";


        // Initalise the map
        fstream mapfile;
        mapfile.open("map.txt", ios::in);
        if (mapfile.is_open()) {
            char val;
            int row = 0, col = 0;
            while (!mapfile.eof()) {

                // If not valid digit ASCII ignore
                val = mapfile.get();
                if (val == '0' || val == '1') {

                    map[row][col] = (int) val - (int) '0';

                    col++;
                    if (col % COLUMN == 0) {
                        row++;
                    }
                    col = col % COLUMN;

                }
            }
            mapfile.close();
        }

        // Plan the path
        plan(map);

        int r = goal.first, c = goal.second;

        while (!(node[r][c].parent_i == r && node[r][c].parent_j == c)) {
            cout << "(" << r << ", " << c << ") <-";
            r = node[r][c].parent_i;
            c = node[r][c].parent_j;
        }

    }

    bool isDestination(int row, int col, pair<int, int> dest) {
        if (row == dest.first && col == dest.second)
            return (true);
        else
            return (false);
    }


    bool isValid(int row, int col) {
        return (row >= 0) && (row < ROW) && (col >= 0)
               && (col < COLUMN);
    }

    bool isUnBlocked(int map[][COLUMN], int row, int col) {
        // Returns true if the cell is not blocked else false
        if (map[row][col] == 0)
            return (true);
        else
            return (false);
    }

    double calculateHValue(int row, int col, pair<int, int> goal) {
        // Return using the distance formula
        return ((double) sqrt(
                (row - goal.first) * (row - goal.first)
                + (col - goal.second) * (col - goal.second)));
    }

    bool plan(int map[][COLUMN]) {

        if (isValid(src.first, src.second)) {

            // If the destination is out of range
            if (isValid(goal.first, goal.second)) {

                // Either the source or the destination is blocked
                if (!isUnBlocked(map, src.first, src.second)
                    || !isUnBlocked(map, goal.first, goal.second)) {
                    printf("Source or the destination is blocked. \n");
                    return false;
                }

                // If the destination cell is the same as source cell
                if (isDestination(src.first, src.second, goal)) {
                    printf("We are already at the destination\n");
                    return false;
                }

                bool closedList[ROW][COLUMN];
                memset(closedList, 0, sizeof(closedList));

                // Initialise all nodes in the map
                int i, j;
                for (i = 0; i < ROW; i++) {
                    for (j = 0; j < COLUMN; j++) {
                        node[i][j].f = FLT_MAX;
                        node[i][j].g = FLT_MAX;
                        node[i][j].h = FLT_MAX;
                        node[i][j].parent_i = -1;
                        node[i][j].parent_j = -1;
                    }
                }

                // Initialising the parameters of the starting node
                i = src.first, j = src.second;
                node[i][j].f = 0.0;
                node[i][j].g = 0.0;
                node[i][j].h = 0.0;
                node[i][j].parent_i = i;
                node[i][j].parent_j = j;

                set < pair < int, pair < int, int>>> openList;

                openList.insert(make_pair(0.0, make_pair(i, j)));

                bool reachedDest = 0;

                while (!openList.empty()) {
                    pair<int, pair<int, int>> curr_node = *openList.begin();
                    openList.erase(openList.begin());

                    i = curr_node.second.first;
                    j = curr_node.second.second;
                    closedList[i][j] = true;

                    double gTemp, hTemp, fTemp;

                    // 1
                    if (isValid(i - 1, j)) {
                        if (isDestination(i - 1, j, goal)) {
                            node[i - 1][j].parent_i = i;
                            node[i - 1][j].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return true;
                        } else if (!closedList[i - 1][j] && isUnBlocked(map, i - 1, j)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i - 1, j, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i - 1][j].f == FLT_MAX || node[i - 1][j].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i - 1, j)));

                                node[i - 1][j].parent_i = i;
                                node[i - 1][j].parent_j = j;
                                node[i - 1][j].g = gTemp;
                                node[i - 1][j].h = hTemp;
                                node[i - 1][j].f = fTemp;
                            }

                        }
                    }

                    // 2
                    if (isValid(i + 1, j)) {
                        if (isDestination(i + 1, j, goal)) {
                            node[i + 1][j].parent_i = i;
                            node[i + 1][j].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return true;
                        } else if (!closedList[i + 1][j] && isUnBlocked(map, i + 1, j)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i + 1, j, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i + 1][j].f == FLT_MAX || node[i + 1][j].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i + 1, j)));

                                node[i + 1][j].parent_i = i;
                                node[i + 1][j].parent_j = j;
                                node[i + 1][j].g = gTemp;
                                node[i + 1][j].h = hTemp;
                                node[i + 1][j].f = fTemp;
                            }

                        }
                    }

                    // 3
                    if (isValid(i, j - 1)) {
                        if (isDestination(i, j - 1, goal)) {
                            node[i][j - 1].parent_i = i;
                            node[i][j - 1].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return true;
                        } else if (!closedList[i][j - 1] && isUnBlocked(map, i, j - 1)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i, j - 1, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i][j - 1].f == FLT_MAX || node[i][j - 1].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i, j - 1)));

                                node[i][j - 1].parent_i = i;
                                node[i][j - 1].parent_j = j;
                                node[i][j - 1].g = gTemp;
                                node[i][j - 1].h = hTemp;
                                node[i][j - 1].f = fTemp;
                            }

                        }
                    }

                    // 4
                    if (isValid(i, j + 1)) {
                        if (isDestination(i, j + 1, goal)) {
                            node[i][j + 1].parent_i = i;
                            node[i][j + 1].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return reachedDest;
                        } else if (!closedList[i][j + 1] && isUnBlocked(map, i, j + 1)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i, j + 1, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i][j + 1].f == FLT_MAX || node[i][j + 1].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i, j + 1)));

                                node[i][j + 1].parent_i = i;
                                node[i][j + 1].parent_j = j;
                                node[i][j + 1].g = gTemp;
                                node[i][j + 1].h = hTemp;
                                node[i][j + 1].f = fTemp;
                            }
                        }
                    }

                    // 5
                    if (isValid(i - 1, j - 1) && isUnBlocked(map, i-1, j) && isUnBlocked(map, i, j-1)) {
                        if (isDestination(i - 1, j - 1, goal)) {
                            node[i - 1][j - 1].parent_i = i;
                            node[i - 1][j - 1].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return reachedDest;
                        } else if (!closedList[i - 1][j - 1] && isUnBlocked(map, i - 1, j - 1)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i - 1, j - 1, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i - 1][j - 1].f == FLT_MAX || node[i - 1][j - 1].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i - 1, j - 1)));

                                node[i - 1][j - 1].parent_i = i;
                                node[i - 1][j - 1].parent_j = j;
                                node[i - 1][j - 1].g = gTemp;
                                node[i - 1][j - 1].h = hTemp;
                                node[i - 1][j - 1].f = fTemp;
                            }

                        }
                    }

                    // 6
                    if (isValid(i - 1, j + 1) && isUnBlocked(map, i-1, j) && isUnBlocked(map, i, j+1)) {
                        if (isDestination(i - 1, j + 1, goal)) {
                            node[i - 1][j + 1].parent_i = i;
                            node[i - 1][j + 1].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return reachedDest;
                        } else if (!closedList[i - 1][j + 1] && isUnBlocked(map, i - 1, j + 1)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i - 1, j + 1, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i - 1][j + 1].f == FLT_MAX || node[i - 1][j + 1].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i - 1, j + 1)));

                                node[i - 1][j + 1].parent_i = i;
                                node[i - 1][j + 1].parent_j = j;
                                node[i - 1][j + 1].g = gTemp;
                                node[i - 1][j + 1].h = hTemp;
                                node[i - 1][j + 1].f = fTemp;
                            }
                        }
                    }

                    // 7
                    if (isValid(i + 1, j - 1) && isUnBlocked(map, i+1, j) && isUnBlocked(map, i, j-1)) {
                        if (isDestination(i + 1, j - 1, goal)) {
                            node[i + 1][j - 1].parent_i = i;
                            node[i + 1][j - 1].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return reachedDest;
                        } else if (!closedList[i + 1][j - 1] && isUnBlocked(map, i + 1, j - 1)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i + 1, j - 1, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i + 1][j - 1].f == FLT_MAX || node[i + 1][j - 1].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i + 1, j - 1)));

                                node[i + 1][j - 1].parent_i = i;
                                node[i + 1][j - 1].parent_j = j;
                                node[i + 1][j - 1].g = gTemp;
                                node[i + 1][j - 1].h = hTemp;
                                node[i + 1][j - 1].f = fTemp;
                            }

                        }
                    }

                    // 8
                    if (isValid(i + 1, j + 1) && isUnBlocked(map, i+1, j) && isUnBlocked(map, i, j+1)) {
                        if (isDestination(i + 1, j + 1, goal)) {
                            node[i + 1][j + 1].parent_i = i;
                            node[i + 1][j + 1].parent_j = j;
                            printf("The destination cell is found\n");
                            // Trace the path
                            reachedDest = 1;
                            return reachedDest;
                        } else if (!closedList[i + 1][j + 1] && isUnBlocked(map, i + 1, j + 1)) {

                            gTemp = node[i][j].g + 1;
                            hTemp = calculateHValue(i + 1, j + 1, goal);
                            fTemp = gTemp + hTemp;

                            // Has this node already been opened?
                            if (node[i + 1][j + 1].f == FLT_MAX || node[i + 1][j + 1].f > fTemp) {

                                openList.insert(make_pair(fTemp, make_pair(i + 1, j + 1)));

                                node[i + 1][j + 1].parent_i = i;
                                node[i + 1][j + 1].parent_j = j;
                                node[i + 1][j + 1].g = gTemp;
                                node[i + 1][j + 1].h = hTemp;
                                node[i + 1][j + 1].f = fTemp;
                            }

                        }
                    }
                }
                if (!reachedDest) {
                    printf("Failed to find the Destination Cell\n");
                    return reachedDest;
                }
            } else {
                printf("Destination is invalid\n");
                return false;
            }
        } else {
            printf("Source is invalid\n");
            return false;
        }
        return false;
    }
};


int main(int argc, char **argv) {
    ros::init(argc, argv, "astar");
    Astar astar;
    ros::spin();
    return 0;
}
