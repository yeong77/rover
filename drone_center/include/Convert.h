#pragma once

#include <tf/tf.h>

double* quaternion_to_euler(double x, double y, double z, double w);
double* euler_to_quaternion(double roll, double pitch, double yaw);
double degree_to_radian(double degree);
double radian_to_degree(double radian);
double _norm(double* _goal, geometry_msgs::PoseStamped _pose);
void _transform(double* pose);