#ifndef KINEMATIC_H

#define KINEMATIC_H

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#define LEG_LEFT 1
#define LEG_RIGHT 0

#define DIR_X 1
#define DIR_Y 2  
#define DIR_Z 3

class Kinematic
{
	public:
		Kinematic();
		bool pToJL(Eigen::Vector3d trans ,Eigen::Vector3d rot,std::vector<double>&joints);
		bool pToJR(Eigen::Vector3d trans ,Eigen::Vector3d rot,std::vector<double>&joints);
		bool getPosL(std::vector<double> &trans,std::vector<double>&rot);
		bool getPosR(std::vector<double> &trans,std::vector<double>&rot);
	private:
		robot_model::RobotModelPtr kinematic_model;
		robot_model_loader::RobotModelLoader *robot_model_loader;//("robot_description");
		robot_state::RobotState* kinematic_state;
		robot_state::JointModelGroup* joint_model_group;
		robot_state::JointModelGroup* joint_model_group1;
		std::vector<std::string> joint_names;
		std::vector<std::string> joint_names1;
};


class Stand
{
	public:
		Stand(Kinematic* _k1,double _delta,size_t count=100);
		int execute(std::vector<float> &lPoses,std::vector<float> &rPoses);
		int getPose();
	private:
		size_t executeCount;
		size_t count;
		double dh;
		double delta;
		Kinematic* k1;
		Eigen::Vector3d lTrans;
    	Eigen::Vector3d lRot;
   		Eigen::Vector3d rTrans;
    	Eigen::Vector3d rRot;

};

class Ready
{
		
	public:
		Ready(Kinematic* _k1,double _delta,size_t _count=100);
		int execute(std::vector<float> &lPoses,std::vector<float> &rPoses);
		int getPose();
	private:
		size_t executeCount;
		size_t count;
		double dh;
		double delta;
		Kinematic* k1;
		Eigen::Vector3d lTrans;
    	Eigen::Vector3d lRot;
   		Eigen::Vector3d rTrans;
    	Eigen::Vector3d rRot;
};


class LegAct
{
	public:
		LegAct(Kinematic* _k1,int _whichLeg,size_t _count);
		int init(int _which,double _distance);
		int resetOrg();
		int execute(std::vector<float> &poses);
	private:
		Kinematic* k1;
		int whichLeg;
		size_t executeCount;
		size_t count;
		double dx;
		double dy;
		double dz;
		double deltaX;
		double deltaY;
		double deltaZ;
		Eigen::Vector3d trans;
    	Eigen::Vector3d rot;


};




#endif