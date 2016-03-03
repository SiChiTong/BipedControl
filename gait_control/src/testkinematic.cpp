#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <kinematic.h>


int main(int argc, char **argv)
{
	ros::init(argc, argv, "right_arm_kinematics");
	ros::AsyncSpinner spinner(1);
  	spinner.start();

	Kinematic k1;

	// for(int i=0;i<10;i++)
	// {	

		// Eigen::Vector3d trans(-0.1,-0.1,-0.65-i*0.1/10.0);
		// Eigen::Vector3d rot(0,0,0);
		// std::vector<double> result;
		// bool success=k1.pToJL(trans,rot,result);
		// std::cout<<trans<<std::endl;

	//Right Leg
	std::vector<double> trans;
	std::vector<double> rot;
	k1.getPosR(trans,rot);
	std::cout<<"Right Leg"<<std::endl;
	std::cout<<trans[0]<<" ";
	std::cout<<trans[1]<<" ";
	std::cout<<trans[2]<<std::endl;
	std::cout<<rot[0]<<" ";
	std::cout<<rot[1]<<" ";
	std::cout<<rot[2]<<" ";
	std::cout<<rot[3]<<std::endl;

	//Left Leg
	k1.getPosL(trans,rot);
	std::cout<<"Left Leg"<<std::endl;
	std::cout<<trans[0]<<" ";
	std::cout<<trans[1]<<" ";
	std::cout<<trans[2]<<std::endl;
	std::cout<<rot[0]<<" ";
	std::cout<<rot[1]<<" ";
	std::cout<<rot[2]<<" ";
	std::cout<<rot[3]<<std::endl;

	// }
	ros::shutdown();
	return 0;
}