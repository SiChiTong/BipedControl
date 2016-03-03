

#include <ros/ros.h>


// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


//data file
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include "gtest/gtest.h"
#include <iostream>

//geometry
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <gazebo/math/Quaternion.hh>
#include <math.h>
#include <gazebo_msgs/ModelState.h>
#include <std_msgs/Float64.h>
#include "param.h"
#include <vector>
#include <nav_msgs/Odometry.h>



//gazebo
#include <gazebo/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/PID.hh>

//jotstick
#include <sensor_msgs/Joy.h>

#include <kinematic.h>


//parameter
#define FRESH_DUR 0.06



//declaration
ros::Publisher pub_l_leg_lax_;
ros::Publisher pub_l_leg_uay_;
ros::Publisher pub_l_leg_kny_;
ros::Publisher pub_l_leg_lhy_;
ros::Publisher pub_l_leg_mhx_;
ros::Publisher pub_l_leg_uhz_;


ros::Publisher pub_r_leg_lax_;
ros::Publisher pub_r_leg_uay_;
ros::Publisher pub_r_leg_kny_;
ros::Publisher pub_r_leg_lhy_;
ros::Publisher pub_r_leg_mhx_;
ros::Publisher pub_r_leg_uhz_;

ros::Subscriber sub_pose_;

std_msgs::Float64 jointRadian;


//motion variale
std::vector<float> lPoses;
std::vector<float> rPoses;

std::vector<float> rlax;
std::vector<float> ruay;
std::vector<float> rkny;
std::vector<float> rlhy;
std::vector<float> rmhx;
std::vector<float> ruhz;

std::vector<float> llax;
std::vector<float> luay;
std::vector<float> lkny;
std::vector<float> llhy;
std::vector<float> lmhx;
std::vector<float> luhz;


const char bagPath[]="/root/mys/src/drcsim/gait_control/src/test.bag";

Kinematic* k1;
Stand* stand1;
Ready* ready1;
LegAct* rLegAct;
LegAct* lLegAct;
ros::Timer timer1;

int initRobot();
int calMass();
int stand();
void callback(const ros::TimerEvent&);
void getPose(const nav_msgs::Odometry::ConstPtr& msg);
void joyCallBack(const sensor_msgs::Joy::ConstPtr& joy);




class Biped
{
  public:
    gazebo::math::Vector3 pose;
    gazebo::math::Quaternion orientation;

    gazebo::math::Vector3 linear;
    gazebo::math::Vector3 angular;
    
};

Biped biped;




int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_joint_trajectory_test");

//  bool wait = true;
//  while (wait)
//  {
//    ros::Time t = ros::Time::now();
//    if (t.toSec() > 0)
//      wait = false;
//  }


// //read data
//   rosbag::Bag bag;
//   bag.open(bagPath, rosbag::bagmode::Read);
//   std::vector<std::string> topics;
//   topics.push_back(std::string("chatter"));

//   rosbag::View view(bag, rosbag::TopicQuery(topics));
//   BOOST_FOREACH(rosbag::MessageInstance const m, view)
//   {
//       std_msgs::String::ConstPtr s = m.instantiate<std_msgs::String>();
//       if (s != NULL)
//       {
//         // std::cout<<s->data<<std::endl;
//           // ASSERT_EQ(s->data, std::string("foo"));
//       }
          
      
//       std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
//       if (i != NULL)
//       {
//           std::cout<<i->data<<std::endl;
//       }
//           // ASSERT_EQ(i->data, 42);
      
//   }

//   bag.close();


  ros::NodeHandle rosnode;
//  ros::Publisher pub_ = rosnode.advertise<trajectory_msgs::JointTrajectory>(
//    "joint_trajectory", 1, true);

//  trajectory_msgs::JointTrajectory jt;

  pub_l_leg_lax_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/l_leg_lax_position_controller/command", 1, true);
  pub_l_leg_uay_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/l_leg_uay_position_controller/command", 1, true);
  pub_l_leg_kny_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/l_leg_kny_position_controller/command", 1, true);
  pub_l_leg_lhy_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/l_leg_lhy_position_controller/command", 1, true);
  pub_l_leg_mhx_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/l_leg_mhx_position_controller/command", 1, true); 
  pub_l_leg_uhz_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/l_leg_uhz_position_controller/command", 1, true);


  pub_r_leg_lax_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/r_leg_lax_position_controller/command", 1, true);
  pub_r_leg_uay_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/r_leg_uay_position_controller/command", 1, true);
  pub_r_leg_kny_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/r_leg_kny_position_controller/command", 1, true);
  pub_r_leg_lhy_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/r_leg_lhy_position_controller/command", 1, true);
  pub_r_leg_mhx_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/r_leg_mhx_position_controller/command", 1, true);
  pub_r_leg_uhz_ =rosnode.advertise<std_msgs::Float64>(
              "/atlas/r_leg_uhz_position_controller/command", 1, true);

  

  ros::Publisher pubModelState_ =rosnode.advertise<gazebo_msgs::ModelState>(
              "gazebo/set_model_state", 1, true);

  gazebo_msgs:: ModelState jtModelState;

//  ros::Publisher pubAtlasCommand = rosnode->advertise<atlas_msgs::AtlasCommand>(
//    "/atlas/atlas_command", 100, true);

  // ros::Subscriber sub = rosnode.subscribe("/ground_truth_odom", 10, getPose);
  // ros::Subscriber sub1 = rosnode.subscribe("/atlas/joint_states", 10, getPose);

  initRobot();

  //init kinematic engine
  

  // stand();

  //modelPosJt
  jtModelState.model_name="atlas";
  jtModelState.pose.position.x=0;
  jtModelState.pose.position.y=0;
  jtModelState.pose.position.z=1;




  //initialize robot
  lPoses[5]=0;
  lPoses[4]=0;
  lPoses[3]=0;//-0.315;
  lPoses[2]=0;//1.170;
  lPoses[1]=0;//-0.855;
  lPoses[0]=0;

  rPoses[5]=0;
  rPoses[4]=0;
  rPoses[3]=0;//-0.315;
  rPoses[2]=0;//1.170;
  rPoses[1]=0;//-0.855;
  rPoses[0]=0;

  jointRadian.data=lPoses[0]; pub_l_leg_lax_.publish(jointRadian);
  jointRadian.data=lPoses[1]; pub_l_leg_uay_.publish(jointRadian);
  jointRadian.data=lPoses[2]; pub_l_leg_kny_.publish(jointRadian);
  jointRadian.data=lPoses[3]; pub_l_leg_lhy_.publish(jointRadian);
  jointRadian.data=lPoses[4]; pub_l_leg_mhx_.publish(jointRadian);
  jointRadian.data=lPoses[5]; pub_l_leg_uhz_.publish(jointRadian);

  jointRadian.data=rPoses[0]; pub_r_leg_lax_.publish(jointRadian);
  jointRadian.data=rPoses[1]; pub_r_leg_uay_.publish(jointRadian);
  jointRadian.data=rPoses[2]; pub_r_leg_kny_.publish(jointRadian);
  jointRadian.data=rPoses[3]; pub_r_leg_lhy_.publish(jointRadian);
  jointRadian.data=rPoses[4]; pub_r_leg_mhx_.publish(jointRadian);
  jointRadian.data=rPoses[5]; pub_r_leg_uhz_.publish(jointRadian);

  sleep(2);
  pubModelState_.publish(jtModelState);


  k1=new Kinematic();
  // lLegAct=new LegAct(k1,LEG_LEFT,50);
  // lLegAct->init(DIR_Z,+0.1);
  // rLegAct=new LegAct(k1,LEG_RIGHT,50);
  // rLegAct->init(DIR_Z,+0.1);
  stand1=new Stand(k1,0.15,50);

  


  timer1=rosnode.createTimer(ros::Duration(FRESH_DUR),callback);

  ros::Subscriber sub = rosnode.subscribe<sensor_msgs::Joy>("joy", 10, joyCallBack);

  ros::spin();






  return 0;
}
 
void joyCallBack(const sensor_msgs::Joy::ConstPtr& joy)
{
  double duration=FRESH_DUR*(1-0.6*joy->axes[0]);
  ROS_INFO("Now the duration is %f",duration);
  timer1.setPeriod(ros::Duration(duration));
  // std::cout<<"1: "<<joy->axes[0]<<" ";
  // std::cout<<"2: "<<joy->axes[1]<<" ";
  // std::cout<<"3: "<<joy->axes[2]<<std::endl;
}

void callback(const ros::TimerEvent&)
{
  static unsigned long tCount=0;
  static float t=0;

  const float f=1.0/10;
  static bool flag=true;
  static bool lastFlag=flag;
  static int count=1;
  static int action=0;

  switch (action)
  {
    case 0:
      if(stand1->execute(lPoses,rPoses)<0)//if(lLegAct->execute(lPoses)==-1||rLegAct->execute(rPoses)==-1)
      {
        ROS_INFO("Stand action is finish");
        action++;
        sleep(1);
        ready1=new Ready(k1,-0.11,50);
        return ;
      }
      break;
    case 1:
      if(ready1->execute(lPoses,rPoses)<0)
      {
        ROS_INFO("Ready1 action is finish");
        action++;
        sleep(1);
        rLegAct=new LegAct(k1,LEG_RIGHT,50);
        lLegAct=new LegAct(k1,LEG_LEFT,50);
        rLegAct->init(DIR_Z,0.09);
        rLegAct->init(DIR_X,0.04);
        return ;
      }
      break;
    case 2:
      if(rLegAct->execute(rPoses)<0)
      {
        ROS_INFO("rLegAct1 action is finish");
        action++;
        // sleep(1);
        rLegAct->resetOrg();
        rLegAct->init(DIR_Z,-0.09);
        rLegAct->init(DIR_X,0.04);
        
        return ;
      }
      break;
    case 3:
      if(rLegAct->execute(rPoses)<0||lLegAct->execute(lPoses)<0)
      {
        ROS_INFO("rLegAct2 action is finish");
        action++;
        // sleep(1);

        rLegAct->resetOrg();
        lLegAct->resetOrg();
        rLegAct->init(DIR_Y,0.22);
        lLegAct->init(DIR_Y,0.22);
        rLegAct->init(DIR_X,-0.08);
        lLegAct->init(DIR_X,-0.08);


        return ;
      }
      break;
    case 4:
      if(rLegAct->execute(rPoses)<0||lLegAct->execute(lPoses)<0)
      {
        ROS_INFO("Ready2 action is finish");
        action++;

        rLegAct->resetOrg();
        lLegAct->resetOrg();
        lLegAct->init(DIR_X,0.08);
        lLegAct->init(DIR_Z,0.09);
      }
      break;
    case 5:
      if(rLegAct->execute(rPoses)<0||lLegAct->execute(lPoses)<0)
      {
        ROS_INFO("Action5 is finish");
        action++;
        rLegAct->resetOrg();
        lLegAct->resetOrg();
        lLegAct->init(DIR_X,0.08);
        lLegAct->init(DIR_Z,-0.09);
      }
      break;
    case 6:
      if(rLegAct->execute(rPoses)<0||lLegAct->execute(lPoses)<0)
      {
        ROS_INFO("Action6 is finish");
        action++;
        rLegAct->resetOrg();
        lLegAct->resetOrg();

        rLegAct->init(DIR_Y,-0.22);
        lLegAct->init(DIR_Y,-0.22);
        rLegAct->init(DIR_X,-0.08);
        lLegAct->init(DIR_X,-0.08);
        // rLegAct->init(DIR_Y,-0.24);
        // lLegAct->init(DIR_Y,-0.24);
        // rLegAct->resetOrg();
        // lLegAct->resetOrg();
      }
      break;

    case 7:
      if(rLegAct->execute(rPoses)<0||lLegAct->execute(lPoses)<0)
      {
        ROS_INFO("Action7 is finish");
        action++;
        rLegAct->resetOrg();
        lLegAct->resetOrg();
        rLegAct->init(DIR_X,0.08);
        rLegAct->init(DIR_Z,0.09);
        // lLegAct->init(DIR_X,-0.2);
        // rLegAct->init(DIR_Z,0.05);
      }
      break;

    case 8:
      if(rLegAct->execute(rPoses)<0||lLegAct->execute(lPoses)<0)
      {
        ROS_INFO("Action8 is finish");
        action++;
        rLegAct->resetOrg();
        lLegAct->resetOrg();
        rLegAct->init(DIR_X,0.08);
        rLegAct->init(DIR_Z,-0.09);
        ROS_INFO("Action7 is finish");
      }
      break;

    case 9:
      if(rLegAct->execute(rPoses)<0||lLegAct->execute(lPoses)<0)
      {
        action=3;
        ROS_INFO("Action9 is finish");

      }
      break;
    default:
      ROS_INFO("No action");
      break;

  }
  // t=tCount*FRESH_DUR;

  // float phase=sin(2*M_PI*f*t);
  // if (phase<0)
  // {
  //   phase=-phase;
  // }
  // ROS_INFO("%f",t);


  // lPoses[0]=rlax[count-1]+phase*(rlax[count]-rlax[count-1]);
  // lPoses[1]=ruay[count-1]+phase*(ruay[count]-ruay[count-1]);
  // lPoses[2]=rkny[count-1]+phase*(rkny[count]-rkny[count-1]);
  // lPoses[3]=rlhy[count-1]+phase*(rlhy[count]-rlhy[count-1]);
  // lPoses[4]=rmhx[count-1]+phase*(rmhx[count]-rmhx[count-1]);
  // lPoses[5]=ruhz[count-1]+phase*(ruhz[count]-ruhz[count-1]);

  // rPoses[0]=llax[count-1]+phase*(llax[count]-llax[count-1]);
  // rPoses[1]=luay[count-1]+phase*(luay[count]-luay[count-1]);
  // rPoses[2]=lkny[count-1]+phase*(lkny[count]-lkny[count-1]);
  // rPoses[3]=llhy[count-1]+phase*(llhy[count]-llhy[count-1]);
  // rPoses[4]=lmhx[count-1]+phase*(lmhx[count]-lmhx[count-1]);
  // rPoses[5]=luhz[count-1]+phase*(luhz[count]-luhz[count-1]);

  jointRadian.data=lPoses[0]; pub_l_leg_lax_.publish(jointRadian);
  jointRadian.data=lPoses[1]; pub_l_leg_uay_.publish(jointRadian);
  jointRadian.data=lPoses[2]; pub_l_leg_kny_.publish(jointRadian);
  jointRadian.data=lPoses[3]; pub_l_leg_lhy_.publish(jointRadian);
  jointRadian.data=lPoses[4]; pub_l_leg_mhx_.publish(jointRadian);
  jointRadian.data=lPoses[5]; pub_l_leg_uhz_.publish(jointRadian);

  jointRadian.data=rPoses[0]; pub_r_leg_lax_.publish(jointRadian);
  jointRadian.data=rPoses[1]; pub_r_leg_uay_.publish(jointRadian);
  jointRadian.data=rPoses[2]; pub_r_leg_kny_.publish(jointRadian);
  jointRadian.data=rPoses[3]; pub_r_leg_lhy_.publish(jointRadian);
  jointRadian.data=rPoses[4]; pub_r_leg_mhx_.publish(jointRadian);
  jointRadian.data=rPoses[5]; pub_r_leg_uhz_.publish(jointRadian);
  






  // ROS_INFO("%f %f",t*0.01,phase);

  tCount++;
}

void getPose(const nav_msgs::Odometry::ConstPtr& msg)
{
  biped.pose.x=msg->pose.pose.position.x;
  biped.pose.y=msg->pose.pose.position.y;
  biped.pose.z=msg->pose.pose.position.z;

  biped.orientation.x=msg->pose.pose.orientation.x;
  biped.orientation.y=msg->pose.pose.orientation.y;
  biped.orientation.z=msg->pose.pose.orientation.z;
  biped.orientation.w=msg->pose.pose.orientation.w;

  gazebo::math::Vector3 result=biped.orientation.GetAsEuler();


  // ROS_INFO("POSITION:%f %f %f",biped.pose.x,biped.pose.y,biped.pose.z);
  // ROS_INFO("POSITION:%f %f %f",result.x,result.y,result.z);
}

int initRobot()
{
  //motion variale

  //buffer 
  lPoses.push_back(0);
  lPoses.push_back(0);
  lPoses.push_back(0);
  lPoses.push_back(0);
  lPoses.push_back(0);
  lPoses.push_back(0);

  rPoses.push_back(0);
  rPoses.push_back(0);
  rPoses.push_back(0);
  rPoses.push_back(0);
  rPoses.push_back(0);
  rPoses.push_back(0);




  // //1
  // rlax.push_back(-0);
  // ruay.push_back(0);
  // rkny.push_back(0);
  // rlhy.push_back(0);
  // rmhx.push_back(0);
  // ruhz.push_back(0);

  // llax.push_back(0);
  // luay.push_back(0);
  // lkny.push_back(0);
  // llhy.push_back(0);
  // lmhx.push_back(-0);
  // luhz.push_back(0);

  // //2
  // rlax.push_back(-0);
  // ruay.push_back(0);
  // rkny.push_back(M_PI/6);
  // rlhy.push_back(-M_PI/6);
  // rmhx.push_back(0);
  // ruhz.push_back(0);

  // llax.push_back(0);
  // luay.push_back(0);
  // lkny.push_back(0);
  // llhy.push_back(0);
  // lmhx.push_back(-0);
  // luhz.push_back(0);

  // //3
  // rlax.push_back(0);
  // ruay.push_back(0);
  // rkny.push_back(M_PI/12);
  // rlhy.push_back(-M_PI/12);
  // rmhx.push_back(0);
  // ruhz.push_back(0);

  // llax.push_back(0);
  // luay.push_back(0);
  // lkny.push_back(-M_PI/12);
  // llhy.push_back(M_PI/12);
  // lmhx.push_back(0);
  // luhz.push_back(0);





}

int stand()
{
  // std::vector<double> trans;
  // std::vector<double> rot;
  // k1.getPosR(trans,rot);
  // k1.getPosL(trans,rot);

    //leftleg
    Eigen::Vector3d trans(0,0.089,-0.7);
    Eigen::Vector3d rot(0,0,0);
    std::vector<double> result;
    bool success;
    success=k1->pToJL(trans,rot,result);
    // std::cout<<result.size()<<std::endl;
    for(int i=0;i<6;i++)
    {
      lPoses[i]=result[5-i];
    }


    //rightleg
    trans[1]=-0.089;
    success=k1->pToJR(trans,rot,result);
    for(int i=0;i<6;i++)
    {
      rPoses[i]=result[5-i];
    }
    // std::cout<<result.size()<<std::endl;


    return 0;
}