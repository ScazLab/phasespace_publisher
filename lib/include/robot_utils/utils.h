/**
 * Copyright (C) 2017 Social Robotics Lab, Yale University
 * Author: Alessandro Roncone
 * email:  alessandro.roncone@yale.edu
 * website: www.scazlab.yale.edu
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU Lesser General Public License, version 2.1 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
**/

#ifndef __UTILS_H__
#define __UTILS_H__

#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>

#include <sensor_msgs/JointState.h>

// #include "human_robot_collaboration_msgs/DoAction.h"

#define SUBSCRIBER_BUFFER 3

// #define THREAD_FREQ       100 // [Hz]

// Allowed default states for the system
#define RECOVER       -4
#define KILLED        -3
#define STOPPED       -2
#define ERROR         -1
#define START          0
#define WORKING        1
#define DONE           2
#define CTRL_RUNNING  10
#define CTRL_DONE     11
#define CTRL_FAIL     12

// // Both arms
// #define ACTION_HOME         human_robot_collaboration_msgs::DoAction::Request::ACTION_HOME
// #define ACTION_RELEASE      human_robot_collaboration_msgs::DoAction::Request::ACTION_RELEASE
// #define ACTION_HAND_OVER    "hand_over"
// // Only left arm
// #define ACTION_GET          "get"
// #define ACTION_PASS         "pass"
// #define ACTION_GET_PASS     "get_pass"
// #define ACTION_CLEANUP      "cleanup"
// // Only right arm
// #define ACTION_HOLD         "hold"
// #define ACTION_START_HOLD   "start_hold"
// #define ACTION_END_HOLD     "end_hold"
// // Protected action keys used for things that are not real actions
// #define LIST_ACTIONS        human_robot_collaboration_msgs::DoAction::Request::LIST_ACTIONS
// #define LIST_OBJECTS        human_robot_collaboration_msgs::DoAction::Request::LIST_OBJECTS

// // Response states to send back to the service
// #define NO_IR_SENSOR    human_robot_collaboration_msgs::DoAction::Response::NO_IR_SENSOR
// #define OBJ_NOT_IN_DB   human_robot_collaboration_msgs::DoAction::Response::OBJ_NOT_IN_DB
// #define NO_OBJ          human_robot_collaboration_msgs::DoAction::Response::NO_OBJ
// #define ACT_KILLED      human_robot_collaboration_msgs::DoAction::Response::ACT_KILLED
// #define ACT_NOT_IN_DB   human_robot_collaboration_msgs::DoAction::Response::ACT_NOT_IN_DB
// #define ACT_NOT_IMPL    human_robot_collaboration_msgs::DoAction::Response::ACT_NOT_IMPL
// #define ACT_FAILED      human_robot_collaboration_msgs::DoAction::Response::ACT_FAILED
// #define INV_KIN_FAILED  human_robot_collaboration_msgs::DoAction::Response::INV_KIN_FAILED
// #define CHECK_OBJ_IDS   "check_obj_ids"

// #define Z_HIGH         0.400
// #define Z_LOW          0.200

// #define ARM_SPEED      0.120    // [m/s]
// #define ARM_ROT_SPEED  0.500    // [rad/s] ?

// #define FORCE_THRES_R       2.0  // [N]
// #define FORCE_THRES_L       2.0  // [N]
// #define FORCE_ALPHA         0.2
// #define FILTER_EPSILON     1e-6
// #define FORCE_FILT_VAR_L  0.001
// #define FORCE_FILT_VAR_R   0.01
// #define REL_FORCE_THRES_L  65.0
// #define REL_FORCE_THRES_R 500.0

// #define HORIZONTAL_ORI_L      0.0, 0.70, 0.10, 0.70
// #define VERTICAL_ORI_L        0.0,  1.0,  0.0,  0.0

// #define HORIZONTAL_ORI_R     -0.590, 0.240, -0.298, 0.711
// #define HANDOVER_ORI_R        0.0, 0.7, 0.7, 0.0

// #define VERTICAL_ORI_R        0.0, 1.0, 0.0, 0.0
// #define VERTICAL_ORI_R2       0.1, 1.0, 0.0, 0.0

// #define POOL_POS_L  -0.05,  0.85, 0.30
// #define POOL_ORI_L   -0.7,   0.7,  0.0, 0.0
// #define POOL_POS_R  -0.05, -0.75, 0.01
// #define POOL_ORI_R    0.7,   0.7,  0.0, 0.0

// #define HOME_POS_L   0.65,  0.45
// #define HOME_POS_R   0.65, -0.25

// #define EPSILON         1e-8

// #define OBJ_NOT_FOUND_NUM_ATTEMPTS  30

/*
 * sets the position of a pose
 *
 * @param     pose, and three floats indicating the 3D position
 *
 * return     N/A
 */
void setPosition(geometry_msgs::Pose& pose, float x, float y, float z);

/*
 * sets the orientation of a pose
 *
 * @param     pose, and four floats indicating the 4D orientation quaternion
 *
 * return     N/A
 */
void setOrientation(geometry_msgs::Pose& pose, float x, float y, float z, float w);

/*
 * converts an integer to a string
 *
 * @param      integer to be converted
 * return     converted string
 */
std::string toString(const int a );

/**
 * converts a vector of integers to a string
 *
 * @param  _v vector of integers to be converted
 * @return    converted string
 */
std::string toString(std::vector<int> const& _v);

/*
 * converts an double to a string
 *
 * @param      double to be converted
 * return     converted string
 */
std::string toString(const double a );

/**
 * converts a vector of double to a string
 *
 * @param  _v vector of double to be converted
 * @return    converted string
 */
std::string toString(std::vector<double> const& _v);

/**
 * converts a geometry_msgs/Pose to a string
 *
 * @param  _p pose to be converted
 * @return    converted string
 */
std::string toString(const geometry_msgs::Pose& _p);

/**
 * Norm of a vector
 *
 * @param  vector<double> the 3D point as vector
 * @return                the norm of the vector
 */
double norm(std::vector<double> const& _v);

/**
 * Norm of a geometry_msgs::Point
 *
 * @param  geometry_msgs::Point the 3D point
 * @return                      the norm of the point
 */
double norm(const geometry_msgs::Point & _v);

/**
 * Operator + (sum) between two geometry_msgs::Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   the sum of the two
 */
geometry_msgs::Point operator+ (const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Operator - (difference) between two geometry_msgs:Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   the difference of the two (i.e. a - b)
 */
geometry_msgs::Point operator- (const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Operator == (equality) between two geometry_msgs::Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   true/false if a==b or not
 */
bool                 operator==(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Operator * (addition)       between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to sum a with
 * @return   the element-by-element sum of a with b
 */
geometry_msgs::Point operator+ (const geometry_msgs::Point& a, const double& b);

/**
 * Operator - (subtraction)    between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to subtract a with
 * @return   the element-by-element subtraction of a with b
 */
geometry_msgs::Point operator- (const geometry_msgs::Point& a, const double& b);

/**
 * Operator * (multiplication) between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to multiply a with
 * @return   the element-by-element multiplication of a with b
 */
geometry_msgs::Point operator* (const geometry_msgs::Point& a, const double& b);

/**
 * Operator / (division)       between a geometry_msgs::Point and a double
 *
 * @param  a the first point
 * @param  b the double to divide a with
 * @return   the element-by-element division of a with b
 */
geometry_msgs::Point operator/ (const geometry_msgs::Point& a, const double& b);

/**
 * Dot, or scalar, product between two geometry_msgs::Points
 *
 * @param  a the first point
 * @param  b the second point
 * @return   the dot product
 */
double dot(const geometry_msgs::Point& a, const geometry_msgs::Point& b);

/**
 * Assigns a set of doubles to a quaternion. It is not possible to create the
 * assignment operator as a free function, so I had to resort to this hack
 *
 * @param q The quaternion to assign the doubles to
 * @param x The new x value of the quaternion
 * @param y The new y value of the quaternion
 * @param z The new z value of the quaternion
 * @param w The new w value of the quaternion
 */
void quaternionFromDoubles(geometry_msgs::Quaternion &q,
                           double x, double y, double z, double w);
/**
 * Print function for a geometry_msgs::Point.
 *
 * @return A text description of the Point.
 */
std::string print(geometry_msgs::Point p);

/**
 * Print function for a geometry_msgs::Quaternion.
 *
 * @return A text description of the Quaternion.
 */
std::string print(geometry_msgs::Quaternion q);

/**
 * Print function for a geometry_msgs::Pose.
 *
 * @return A text description of the Pose.
 */
std::string print(geometry_msgs::Pose p);

/**
 * Struct that wrap a generic state and provides useful methods around it
 */
struct State
{
private:
    int state;

public:

    /**
     * Constructor, with default initializations of the state and time
     *
     * @param _state the new state
     */
    explicit State(int _state = START) : state(_state) { };

    /**
     * Sets the state to a new state. Updates the time accordingly.
     *
     * @param _state the new state
     */
    void set(int _state);

    /**
     * Returns the state as an integer
     */
    /* implicit */
    operator int();

    /**
     * Returns the state as a std::string (i.e. a text description of the state)
     */
    /* implicit */
    operator std::string();
};

#endif
