#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState



if __name__ == '__main__':
    
    while 1:
        try:
            rospy.wait_for_service('/gazebo/get_model_state')
            qw = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            resp1=qw("atlas","atlas::l_uglut");
            print resp1.success
            print resp1.pose.position

        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

        rospy.timer.sleep(1)
