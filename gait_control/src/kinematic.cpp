#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <kinematic.h>



Kinematic::Kinematic()
{

	robot_model_loader=new robot_model_loader::RobotModelLoader("robot_description");
	kinematic_model = robot_model_loader->getModel();

	ROS_INFO("Model frame: %s", kinematic_model->getModelFrame().c_str());
	kinematic_state=new robot_state::RobotState(kinematic_model);

	kinematic_state->setToDefaultValues();

	joint_model_group = kinematic_model->getJointModelGroup("right_leg");
	joint_model_group1 = kinematic_model->getJointModelGroup("left_leg");
	joint_names = joint_model_group->getJointModelNames(); 
	joint_names1 = joint_model_group1->getJointModelNames();


	//see the joint value
	// std::vector<double> joint_values;
	// kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
	// for(std::size_t i = 0; i < joint_names.size(); ++i)
 //  	{
 //    	ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
 //  	}

  	// //set joint value
  	// joint_values[0] = 1.57;
 	 // kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
 	 // kinematic_state->enforceBounds();//constraint
 	 // ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));


  	// Forward Kinematics
    // kinematic_state->setToRandomPositions(joint_model_group);
  	// kinematic_state->enforceBounds();
  	// const Eigen::Affine3d &end_effector_statekinematic_state->getGlobalLinkTransform("r_foot");

}

bool Kinematic::getPosL(std::vector<double> &trans,std::vector<double>&rot)
{
	const Eigen::Affine3d &current_State=kinematic_state->getGlobalLinkTransform("l_foot");
	// ROS_INFO_STREAM("Translation: " << current_State.translation());
 //  	ROS_INFO_STREAM("Rotation: " << current_State.rotation()); 
	Eigen::Transform<double, 3, 2>::ConstTranslationPart t1=current_State.translation();
	Eigen::Affine3d::LinearMatrixType  r1=current_State.rotation();

  	Eigen::AngleAxisd angleAxis(r1);
  	trans.clear();
  	trans.push_back(t1[0]);
  	trans.push_back(t1[1]);
  	trans.push_back(t1[2]);
  	
  	rot.clear();
  	// Eigen::Vector3 &
  	// rot.push_back(angleAxis.angle());
  	rot.push_back(angleAxis.axis()[0]*angleAxis.angle());
  	rot.push_back(angleAxis.axis()[1]*angleAxis.angle());
  	rot.push_back(angleAxis.axis()[2]*angleAxis.angle());
	return true;
}

bool Kinematic::getPosR(std::vector<double> &trans,std::vector<double> &rot)
{
	const Eigen::Affine3d &current_State=kinematic_state->getGlobalLinkTransform("r_foot");

	Eigen::Transform<double, 3, 2>::ConstTranslationPart t1=current_State.translation();
	Eigen::Affine3d::LinearMatrixType  r1=current_State.rotation();

  	Eigen::AngleAxisd angleAxis(r1);
  	trans.clear();
  	trans.push_back(t1[0]);
  	trans.push_back(t1[1]);
  	trans.push_back(t1[2]);
  	
  	rot.clear();
  	// Eigen::Vector3 &
  	// rot.push_back(angleAxis.angle());
  	rot.push_back(angleAxis.axis()[0]*angleAxis.angle());
  	rot.push_back(angleAxis.axis()[1]*angleAxis.angle());
  	rot.push_back(angleAxis.axis()[2]*angleAxis.angle());

	return true;
}
bool Kinematic::pToJR(Eigen::Vector3d trans ,Eigen::Vector3d rot,std::vector<double>&joints)
{


	// Eigen::Affine3d rx =Eigen::Affine3d(Eigen::AngleAxisd(0.0*M_PI, rot));
 //  	Eigen::Affine3d t(Eigen::Translation3d(trans));


  	Eigen::Affine3d rx =Eigen::Affine3d(Eigen::AngleAxisd(rot.x(), Eigen::Vector3d(1,0,0)));
  	Eigen::Affine3d ry =Eigen::Affine3d(Eigen::AngleAxisd(rot.y(), Eigen::Vector3d(0,1,0)));
  	Eigen::Affine3d rz =Eigen::Affine3d(Eigen::AngleAxisd(rot.z(), Eigen::Vector3d(0,0,1)));
  	Eigen::Affine3d rxyz=rz*ry*rx;
  	Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(trans.x(),trans.y(),trans.z())));  
  	Eigen::Affine3d tr=t*rxyz; 

    const Eigen::Affine3d &end_effector_state =tr;

    //  //see the descripte
    //  const Eigen::Affine3d &current_State=kinematic_state->getGlobalLinkTransform("r_foot");
  	// ROS_INFO_STREAM("Translation: " << current_State.translation());
  	// ROS_INFO_STREAM("Rotation: " << current_State.rotation());  


	bool found_ik = kinematic_state->setFromIK(joint_model_group, end_effector_state, 10, 0.1);	
	// Eigen::Affine3d &current_State=kinematic_state->getGlobalLinkTransform("r_foot");



	std::vector<double> joint_values;
	if (found_ik)
	{
		kinematic_state->copyJointGroupPositions(joint_model_group, joint_values);
		for(std::size_t i=0; i < joint_names.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
			// result.push_back(joint_values[i]);
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution");
	}

	joints=joint_values;
	return found_ik;

}


bool Kinematic::pToJL(Eigen::Vector3d trans ,Eigen::Vector3d rot,std::vector<double>&joints)
{


	// Eigen::Affine3d rx =Eigen::Affine3d(Eigen::AngleAxisd(0.0*M_PI, rot));
 //  	Eigen::Affine3d t(Eigen::Translation3d(trans));


  	Eigen::Affine3d rx =Eigen::Affine3d(Eigen::AngleAxisd(rot.x(), Eigen::Vector3d(1,0,0)));
  	Eigen::Affine3d ry =Eigen::Affine3d(Eigen::AngleAxisd(rot.y(), Eigen::Vector3d(0,1,0)));
  	Eigen::Affine3d rz =Eigen::Affine3d(Eigen::AngleAxisd(rot.z(), Eigen::Vector3d(0,0,1)));
  	Eigen::Affine3d rxyz=rz*ry*rx;
  	Eigen::Affine3d t(Eigen::Translation3d(Eigen::Vector3d(trans.x(),trans.y(),trans.z())));  
  	Eigen::Affine3d tr=t*rxyz; 

    const Eigen::Affine3d &end_effector_state =tr;

    //  //see the descripte
    //  const Eigen::Affine3d &current_State=kinematic_state->getGlobalLinkTransform("r_foot");
  	// ROS_INFO_STREAM("Translation: " << current_State.translation());
  	// ROS_INFO_STREAM("Rotation: " << current_State.rotation());  


	bool found_ik = kinematic_state->setFromIK(joint_model_group1, end_effector_state, 10, 0.1);	
	// Eigen::Affine3d &current_State=kinematic_state->getGlobalLinkTransform("r_foot");



	std::vector<double> joint_values;
	if (found_ik)
	{
		kinematic_state->copyJointGroupPositions(joint_model_group1, joint_values);
		for(std::size_t i=0; i < joint_names1.size(); ++i)
		{
			ROS_INFO("Joint %s: %f", joint_names1[i].c_str(), joint_values[i]);
			// result.push_back(joint_values[i]);
		}
	}
	else
	{
		ROS_INFO("Did not find IK solution");
	}


	joints=joint_values;
	return found_ik;

}


Stand::Stand(Kinematic* _k1,double _delta,size_t _count)
{
	executeCount=0;
	count=_count;
	delta=_delta;
	dh=delta/count;

	k1=_k1;

	//left leg
	std::vector<double> tmpTrans;
	std::vector<double> tmpRot;
  	k1->getPosL(tmpTrans,tmpRot);
  	lTrans[0]=tmpTrans[0];
  	lTrans[1]=tmpTrans[1];
  	lTrans[2]=tmpTrans[2];

  	lRot[0]=tmpRot[0];
  	lRot[1]=tmpRot[1];
  	lRot[2]=tmpRot[2];

  	std::cout<<"lRot: "<<lRot<<std::endl;
  	//right leg
  	std::vector<double> tmpTrans1;
	std::vector<double> tmpRot1;
  	k1->getPosR(tmpTrans1,tmpRot1);
  	
  	rTrans[0]=tmpTrans1[0];
  	rTrans[1]=tmpTrans1[1];
  	rTrans[2]=tmpTrans1[2];


  	rRot[0]=tmpRot1[0];
  	rRot[1]=tmpRot1[1];
  	rRot[2]=tmpRot1[2];
  	std::cout<<"rRot: "<<rRot<<std::endl;

}

int Stand::execute(std::vector<float> &lPoses,std::vector<float> &rPoses)
{

	if(executeCount==count)
	{
		return -1;
	}
	else
	{
		executeCount++;
		//increase the step
		lTrans[2]=lTrans[2]+dh;
		rTrans[2]=rTrans[2]+dh;
	}
	//increase the step
	
	std::cout<<"lTrans: "<<lTrans<<std::endl;
	std::cout<<"rTrans: "<<rTrans<<std::endl;
    std::vector<double> result;
    bool success;

    //leftleg
    success=k1->pToJL(lTrans,lRot,result);
 	if(success==false)
 	{
 		return -1;
 	}     
 	else
 	{
	    for(int i=0;i<6;i++)
	    {
	      lPoses[i]=result[5-i];
	    }
	}

    //rightleg
    // trans[1]=-0.089;
    success=k1->pToJR(rTrans,rRot,result);
    if(success==false)
    {
    	return -1;
    }
    else
    {
	    for(int i=0;i<6;i++)
	    {
	      rPoses[i]=result[5-i];
	    }
	}
    // std::cout<<result.size()<<std::endl;

	return 0;
}

/////////////////////////////////////Ready///////////////////////////////
Ready::Ready(Kinematic* _k1,double _delta,size_t _count)
{
	executeCount=0;
	count=_count;
	delta=_delta;
	dh=delta/count;

	k1=_k1;

	//left leg
	std::vector<double> tmpTrans;
	std::vector<double> tmpRot;
  	k1->getPosL(tmpTrans,tmpRot);
  	lTrans[0]=tmpTrans[0];
  	lTrans[1]=tmpTrans[1];
  	lTrans[2]=tmpTrans[2];

  	lRot[0]=tmpRot[0];
  	lRot[1]=tmpRot[1];
  	lRot[2]=tmpRot[2];

  	std::cout<<"lRot: "<<lRot<<std::endl;
  	//right leg
  	std::vector<double> tmpTrans1;
	std::vector<double> tmpRot1;
  	k1->getPosR(tmpTrans1,tmpRot1);
  	
  	rTrans[0]=tmpTrans1[0];
  	rTrans[1]=tmpTrans1[1];
  	rTrans[2]=tmpTrans1[2];


  	rRot[0]=tmpRot1[0];
  	rRot[1]=tmpRot1[1];
  	rRot[2]=tmpRot1[2];
  	std::cout<<"rRot: "<<rRot<<std::endl;

}

int Ready::execute(std::vector<float> &lPoses,std::vector<float> &rPoses)
{

	if(executeCount==count)
	{
		return -1;
	}
	else
	{
		executeCount++;
		//increase the step
		lTrans[1]=lTrans[1]+dh;
		rTrans[1]=rTrans[1]+dh;
	}
	//increase the step
	
	std::cout<<"lTrans: "<<lTrans<<std::endl;
	std::cout<<"rTrans: "<<rTrans<<std::endl;
    std::vector<double> result;
    bool success;

    //leftleg
    success=k1->pToJL(lTrans,lRot,result);
 	if(success==false)
 	{
 		return -1;
 	}     
 	else
 	{
	    for(int i=0;i<6;i++)
	    {
	      lPoses[i]=result[5-i];
	    }
	}

    //rightleg
    // trans[1]=-0.089;
    success=k1->pToJR(rTrans,rRot,result);
    if(success==false)
    {
    	return -1;
    }
    else
    {
	    for(int i=0;i<6;i++)
	    {
	      rPoses[i]=result[5-i];
	    }
	}
    // std::cout<<result.size()<<std::endl;

	return 0;
}


/////////////////////////////LegAct////////////////////////////////////////////
LegAct::LegAct(Kinematic* _k1,int _whichLeg,size_t _count)
{
	// executeCount=0;
	count=_count;
	whichLeg=_whichLeg;
	k1=_k1;
	// dx=0;
	// dy=0;
	// dz=0;

	// deltaX=0;
	// deltaY=0;
	// deltaZ=0;
	resetOrg();



}


int LegAct::init(int _which,double _distance)
{
	switch (_which)
	{
		case DIR_X:
			deltaX=_distance;
			dx=deltaX/count;
			break;

		case DIR_Y:
			deltaY=_distance;
			dy=deltaY/count;
			break;

		case DIR_Z:
			deltaZ=_distance;
			dz=deltaZ/count;
			break;

		default:
			break;
	}

}

int LegAct::resetOrg()
{

	executeCount=0;
	dx=0;
	dy=0;
	dz=0;

	deltaX=0;
	deltaY=0;
	deltaZ=0;

	if(whichLeg==LEG_LEFT)
	{
		//left leg
		std::vector<double> tmpTrans;
		std::vector<double> tmpRot;
	  	k1->getPosL(tmpTrans,tmpRot);
	  	trans[0]=tmpTrans[0];
	  	trans[1]=tmpTrans[1];
	  	trans[2]=tmpTrans[2];

	  	rot[0]=tmpRot[0];
	  	rot[1]=tmpRot[1];
	  	rot[2]=tmpRot[2];

	  	// std::cout<<"lRot: "<<lRot<<std::endl;
	}
	else
	{
		//right leg
 	 	std::vector<double> tmpTrans1;
		std::vector<double> tmpRot1;
  		k1->getPosR(tmpTrans1,tmpRot1);
  	
  		trans[0]=tmpTrans1[0];
  		trans[1]=tmpTrans1[1];
  		trans[2]=tmpTrans1[2];


  		rot[0]=tmpRot1[0];
  		rot[1]=tmpRot1[1];
  		rot[2]=tmpRot1[2];
  		// std::cout<<"rRot: "<<rRot<<std::endl;
	}

}


int LegAct::execute(std::vector<float> &poses)
{

	if(executeCount==count)
	{
		return -1;
	}
	else
	{
		executeCount++;
		//increase the step
		trans[0]=trans[0]+dx;
		trans[1]=trans[1]+dy;
		trans[2]=trans[2]+dz;

	}
	//increase the step
	
	
	// std::cout<<"rTrans: "<<rTrans<<std::endl;
    std::vector<double> result;
    bool success;

	if(whichLeg==LEG_LEFT)
	{
		//leftleg
  		success=k1->pToJL(trans,rot,result);
 		if(success==false)
 		{
	 		return -1;
	 	}     
 		else
 		{
 			// std::cout<<"lTrans: "<<lTrans<<std::endl;
	   		for(int i=0;i<6;i++)
	    	{
	    	  poses[i]=result[5-i];
	    	}
		}
	}
	else
	{
		//rightleg
	    success=k1->pToJR(trans,rot,result);
	    if(success==false)
	    {
	    	return -1;
	    }
	    else
	    {
		    for(int i=0;i<6;i++)
		    {
		      poses[i]=result[5-i];
		    }
		}
	}


	return 0;
}