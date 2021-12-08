//
// Created by Vivek Shankar on 12/6/21.
//
#define ROW 20
#define COLUMN 18

#include <stdio.h>
#include <unistd.h>
#include <ros/ros.h>
#include <iostream>
#include <fstream>

#define GetCurrentDir getcwd

using namespace std;

class Astar {

private:
    ros::NodeHandle n;

    int map[20][18];


public:
    Astar() {
        n = ros::NodeHandle("~");

        // Initalise the map
	fstream mapfile;
	mapfile.open("map.txt", ios::in);
	if (mapfile.is_open()) {
	char val;
	int row = 0, col = 0;
        while(!mapfile.eof()) {
		
                // If not valid digit ASCII ignore
                val = mapfile.get();
                if (val == '0' || val == '1') {
		
                    map[row][col] = (int) val - (int) '0';

		    col++;
                    if(col % COLUMN == 0) {
                        row++;
                    }
                    col = col % COLUMN;

		}    
        }
	mapfile.close();
	}


        for (int row = 0; row < ROW; row++) {
            for (int column = 0; column < COLUMN; column++) {
                std::cout<<map[row][column]<<", ";
            }
            std::cout<<endl;
        }

    }


};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "astar");
    Astar astar;
    ros::spin();
    return 0;
}
