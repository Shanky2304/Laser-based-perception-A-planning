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
#include <stack>
#include <cmath>
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Point.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"

using namespace std;

struct node_deets {

    int parent_i, parent_j;
    // f = g + h
    double f, g, h;
    string dir;
};


class Astar {

private:
    ros::NodeHandle n;

    int map[20][18];
    double curr_x, curr_y;
    pair<double, double> start_xy = make_pair(-8.0, -2.0);

    node_deets node[ROW][COLUMN];
    bool done = 0;

    // Meant to store row, col of the cell where the src or goal lies.
    pair<int, int> src;
    pair<int, int> goal;

    pair<int, int> origin = make_pair((ROW / 2), (COLUMN / 2));

    stack <pair<int, int>> route;

    ros::Subscriber pose_truth_sub;
    ros::Publisher drive_pub;
    geometry_msgs::Vector3 rpy;


public:
    Astar() {
        n = ros::NodeHandle("~");

        double goal_x, goal_y;
        n.getParam("/goalx", goal_x);
        n.getParam("/goaly", goal_y);
        cout << "For goal at: (" << goal_x << ", " << goal_y << ")" << endl;

        cout << "Origin: (" << origin.first << " ," << origin.second << ")" << endl;

        // Use co-ordinates and size of map to evaluate the i, j of src and goal;
        src.first = (int) (origin.first - start_xy.second);
        src.second = (int) (origin.second + start_xy.first);

        cout << "Src_ij: (" << src.first << " ," << src.second << ")" << endl;

        goal.first = (int) (origin.first - goal_y);
        goal.second = (int) (origin.second + goal_x);

        cout << "Goal_ij: (" << goal.first << " ," << goal.second << ")" << endl;


        // Initalise the map
        fstream mapfile;
        mapfile.open("map.txt", ios::in);
        if (mapfile.is_open()) {
            char val;
            int row = 0, col = 0;
            while (!mapfile.eof()) {

                // If not a valid digit ignore
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
        if (plan(map)) {

            int r = goal.first, c = goal.second;

            //Print route chain
            while (!(node[r][c].parent_i == r && node[r][c].parent_j == c)) {
                cout << "(" << r << ", " << c << ") ->";
                route.push(make_pair(r, c));
                r = node[r][c].parent_i;
                c = node[r][c].parent_j;
            }
            route.push(make_pair(r, c));
            /*
             * Print A* exploration map, every cell contains the row, col value of its parent.
             * Obstacles or unvisited cells have the value (-1, -1).
            for (int i = 0; i < ROW; i++) {
                for (int j = 0; j < COLUMN; j++) {
                    r = node[i][j].parent_i;
                    c = node[i][j].parent_j;

                    cout << "(" << r << "," << c << "),";
                }
                cout << endl;
            }*/
            string pose_truth_topic = "/base_pose_ground_truth",
                    cmd_vel_topic = "/cmd_vel";
            pose_truth_sub = n.subscribe(pose_truth_topic, 1, &Astar::pose_truth_callback, this);

            drive_pub = n.advertise<geometry_msgs::Twist>(cmd_vel_topic, 10, false);

        } else {
            // Do nothing
        }

    }

    void pose_truth_callback(const nav_msgs::Odometry &odom) {

        if (!done) {

            cout << "In the callback!!" << endl;
            geometry_msgs::Quaternion quat = odom.pose.pose.orientation;
            geometry_msgs::Point position = odom.pose.pose.position;
            geometry_msgs::Twist twist;

            // Get the robot's current orientation
            QuaternionToRPY(quat, rpy);

            curr_x = position.x;
            curr_y = position.y;

            cout << "In callback : " << curr_x << ", " << curr_y << endl;

            if(route.empty()) {
                done = 1;
                return;
            }

            pair<int, int> next_cell = route.top();
            route.pop();

            cout << "Next cell to go to: (" << next_cell.first << ", " << next_cell.second << ")" << endl;

            pair<int, int> curr_cell = make_pair((int) (origin.first - curr_y), (int) (origin.second + curr_x));
            double theta_of_slope = atan((-1 * (next_cell.first - curr_cell.first))
                                         / (next_cell.second - curr_cell.second));

            double rad_to_turn;
            if (rpy.z < 0) {
                rad_to_turn = rpy.z - theta_of_slope;
            } else {
                rad_to_turn = rpy.z + theta_of_slope;
            }
            cout << "Computed rad to turn: " << rad_to_turn << endl;

            if (curr_cell.first == next_cell.first && curr_cell.second == next_cell.second) {
                cout<<"Reached next cell!"<<endl;
                // Stop the car and update next_cell
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                publish_cmd_vel(twist);
                pair<int, int> next_cell = route.top();
                route.pop();
            } else if (abs(rad_to_turn) > 0.01) {
                // We are not facing the next_cell, rotate

                    twist.angular.z = rad_to_turn;
                    twist.linear.x = 0.0;
                    // publish rad_to_turn angular vel in z
                    publish_cmd_vel(twist);
                    ros::Duration(1).sleep();
            } else {
                // We haven't reached the next cell but are facing towards it
                twist.linear.x = 1.0;
                twist.angular.z = 0.0;
                publish_cmd_vel(twist);
                ros::Duration(1).sleep();
            }
        } else {
            cout << "Drove to goal!!";
        }

    }

//    void driveToGoal() {
//
//        geometry_msgs::Twist twist;
//
//        pair<int, int> curr_cell = route.top();
//        route.pop();
//        cout << "Start cell is: (" << curr_cell.first << ", " << curr_cell.second << ")" << endl;
//
//        // Turn towards goal
//        double theta_of_slope = atan((goal.second - curr_y) / (goal.first - curr_x));
//        double rad_to_turn;
//        if (rpy.z < 0) {
//            rad_to_turn = rpy.z - theta_of_slope;
//        } else {
//            rad_to_turn = rpy.z + theta_of_slope;
//        }
//
//        while (!route.empty()) {
//            //Decide which direction we need to go in.
//            pair<int, int> next_cell = route.top();
//            cout << "Next cell to go to: (" << next_cell.first << ", " << next_cell.second << ")" << endl;
//            route.pop();
//            double theta_of_slope = atan((-1 * (next_cell.first - curr_cell.first))
//                                         / (next_cell.second - curr_cell.second));
//            cout << "RPY.z = " << rpy.z << endl;
//            if (rpy.z < 0) {
//                rad_to_turn = rpy.z - theta_of_slope;
//            } else {
//                rad_to_turn = rpy.z + theta_of_slope;
//            }
//            cout << "Computed rad to turn: " << rad_to_turn << endl;
//            //Rotate the robot
//            if (abs(rad_to_turn) > 0.01) {
//
//                twist.angular.z = rad_to_turn * 2;
//
//                // publish rad_to_turn*2 angular vel in z
//                publish_cmd_vel(twist);
//                // Let the robot turn we can safely ignore callbacks while it's turning.
//                ros::Duration(5).sleep();
//                twist.angular.z = 0.0;
//                publish_cmd_vel(twist);
//
//            }
//            //Drive linearly with speed 1 till we reach the next block
//            int r = (int) (origin.first - curr_y);
//            int c = (int) (origin.second + curr_x);
//
//            while (r != next_cell.first && c != next_cell.second) {
//
//                cout << "In cell: (" << curr_x << ", " << curr_y << ")" << endl;
//                twist.linear.x = 1.0;
//                publish_cmd_vel(twist);
//                ros::Duration(1).sleep();
//            }
//            // Reached next cell
//            twist.linear.x = 0.0;
//            publish_cmd_vel(twist);
//            curr_cell = next_cell;
//        }
//    }

    void publish_cmd_vel(geometry_msgs::Twist twist) {

        ROS_INFO_STREAM("Twist.linear.x = " << twist.linear.x);
        ROS_INFO_STREAM("Twist.angular.x = " << twist.angular.z);
        drive_pub.publish(twist);

    }

    // Function for conversion of quaternion to roll pitch and yaw.
    void QuaternionToRPY(const geometry_msgs::Quaternion msg, geometry_msgs::Vector3 &rpy) {
        // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
        tf::Quaternion quat;
        tf::quaternionMsgToTF(msg, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // the found angles are written in a geometry_msgs::Vector3
        rpy.x = roll;
        rpy.y = pitch;
        rpy.z = yaw;
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
                    if (isValid(i - 1, j - 1) && isUnBlocked(map, i - 1, j)
                        && isUnBlocked(map, i, j - 1)) {
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
                    if (isValid(i - 1, j + 1) && isUnBlocked(map, i - 1, j)
                        && isUnBlocked(map, i, j + 1)) {
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
                    if (isValid(i + 1, j - 1) && isUnBlocked(map, i + 1, j)
                        && isUnBlocked(map, i, j - 1)) {
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
                    if (isValid(i + 1, j + 1) && isUnBlocked(map, i + 1, j)
                        && isUnBlocked(map, i, j + 1)) {
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
