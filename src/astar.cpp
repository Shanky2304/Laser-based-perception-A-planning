//
// Created by Vivek Shankar on 12/6/21.
//
#define ROW 20
#define COLUMN 18

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
        if (!fp) {
            cout << "Error, file couldn't be opened" << endl;
            return 1;
        }
        int val;
        for (int row = 0; row < ROW; row++) {
            for (int column = 0; column < COLUMN; column++) {
                // If not valid digit ASCII ignore
                fp >> val;
                if (val == 0 || val == 1)
                    map[row][column] = val;
                if (!fp) {
                    cout << "Error reading file for element " << row << "," << col << endl;
                    return 1;
                }
            }
        }

        for (int row = 0; row < ROW; row++) {
            for (int column = 0; column < COLUMN; column++) {
                std::cout<<map[row][column]<<", ";
            }
            std::cout<<endl;
        }

    }

};
