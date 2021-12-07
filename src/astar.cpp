//
// Created by Vivek Shankar on 12/6/21.
//

#include <ros/ros.h>
#include <iostream>
#include <fstream>

class Astar {

private:
    ros::NodeHandle n;

    int map[20][18];

    double inlier_threshold = 0.07;

public:
    Astar() {
        n = ros::NodeHandle("~");

        // Initalise the map
        ifstream fp("map.txt");
        if (! fp) {
            cout << "Error, file couldn't be opened" << endl;
            return 1;
        }
        for(int row = 0; row < 20; row++) {  // stop loops if nothing to read
            for(int column = 0; column < 18; column++){
                // If not valid digit ASCII ignore
                fp >> map[row][column];
                if ( ! fp ) {
                    cout << "Error reading file for element " << row << "," << col << endl;
                    return 1;
                }
            }
        }
    }

};
