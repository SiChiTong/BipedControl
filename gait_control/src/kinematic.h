#ifndef KINEMATIC_H

#define KINEMATIC_H

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

class Kinematic
{
	public:
		Kinematic();
	private:
		robot_model::RobotModelPtr kinematic_model;
		robot_model_loader::RobotModelLoader robot_model_loader("robot_description");

};

#endif