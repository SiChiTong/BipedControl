#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point
import tf
import numpy as np


# [
# 'atlas::pelvis',1
# 'atlas::l_uglut',2
# 'atlas::l_lglut',3
# 'atlas::l_uleg',4
# 'atlas::l_lleg',5
# 'atlas::l_talus',6
# 'atlas::l_foot',7
# 'atlas::r_uglut',8
# 'atlas::r_lglut',9
# 'atlas::r_uleg',10
# 'atlas::r_lleg',11
# 'atlas::r_talus',12
# 'atlas::r_foot'13
# ]
# class s:

class cData:
    ox=0.0
    oy=0.0
    oz=0.0
    ow=0.0

    px=0.0
    py=0.0
    pz=0.0


class mass:

    def __init__(self,_oxyz,_weight):
        self.quaternion = (0,0,0,0)
        self.oxyz=np.array(_oxyz).reshape(4,1)
        self.weight=_weight 
    def setRotM(self,x,y,z,w):
        self.quaternion=(x,y,z,w)
        self.euler=tf.transformations.euler_from_quaternion(self.quaternion)
        self.rotM = tf.transformations.euler_matrix(self.euler[0], self.euler[1], self.euler[2], 'rxyz')
    def setTransM(self,x,y,z):
        translation=(x,y,z)
        self.translationM=tf.transformations.translation_matrix(translation)
    def getMass(self):
        self.R=np.array(
            [(0.0,0.0,0.0,0.0),
            (0.0,0.0,0.0,0.0),
            (0.0,0.0,0.0,0.0),
            (0.0,0.0,0.0,1.0)]
            )
        self.R[:3,3]=self.translationM[:3,3]
        # print self.translationM[:3,3]
        self.R[:3,:3]=self.rotM[:3,:3]
        # print self.R[3,3]
        self.wxyz=np.dot(self.R,self.oxyz)

        return self.wxyz


def callback(data):
    global linkData
    linkData=data



def timerCallback(event):
    now = rospy.get_rostime()
    print "<<<",now.secs, now.nsecs

    global linkData
    data=linkData
    # now = rospy.get_rostime()
    # print ">>>",now.secs, now.nsecs
    for i in range(13):
        tmpData[i].ox=data.pose[i+1].orientation.x
        tmpData[i].oy=data.pose[i+1].orientation.y
        tmpData[i].oz=data.pose[i+1].orientation.z
        tmpData[i].ow=data.pose[i+1].orientation.w

        tmpData[i].px=data.pose[i+1].position.x
        tmpData[i].py=data.pose[i+1].position.y
        tmpData[i].pz=data.pose[i+1].position.z




    # print data.pose[2].position.x
    sumMass=np.zeros((3,1))

    for i in range(13):
        l[i].setRotM(
            tmpData[i].ox,
            tmpData[i].oy,
            tmpData[i].oz,
            tmpData[i].ow)

        l[i].setTransM(
            tmpData[i].px,
            tmpData[i].py,
            tmpData[i].pz
            )
    # now = rospy.get_rostime()
    # print "<<<",now.secs, now.nsecs

    for i in range(13):
        tmp1=l[i].getMass()
        tmp2=np.zeros((3,1))
        tmp2[:3,0]=tmp1[:3,0]
        sumMass=np.add(sumMass,tmp2*l[i].weight)
        massPos=sumMass/sumWeight
    #     # print i
    #     # print sumMass
    #     # print sumMass
    # now = rospy.get_rostime()
    # print "<<<",now.secs, now.nsecs

    
    now = rospy.get_rostime()
    print "<<<",now.secs, now.nsecs
    print "========"
    print massPos

    msg=Point()
    msg.x=massPos[0]
    msg.y=massPos[1]
    msg.z=massPos[2]
    massPub.publish(msg)





#buffer to save the coming data
tmpData=[]
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())
tmpData.append(cData())


#mass array
l=[]

l.append(mass((0.0111,0,0.0271,1),14.2529))#pelvis',1
l.append(mass((0.00529262,-0.00344732,0.00313046,1),0.5166))#l_uglut',2
l.append(mass((0.0133341,0.0170484,-0.0312052,2),0.69))#l_lglut',3
l.append(mass((0,0,-0.21,1),7.34))#l_uleg',4
l.append(mass((0.001,0,-0.187,1),4.367))#l_lleg',5
l.append(mass((0,0,0,1),4.367))#l_talus',6
l.append(mass((0.027,0,-0.067,1),1.634))#l_foot',7
l.append(mass((0.00529262,0.00344732,0.00313046,1),1.634))#r_uglut',8
l.append(mass((0.0133341,-0.0170484,-0.0312052,1),0.69))#r_lglut',9
l.append(mass((0,0,-0.21,1),7.34))#r_uleg',10
l.append(mass((0.001,0,-0.187,1),4.367))#r_lleg',11
l.append(mass((0,0,0,1),0.1))#r_talus',12
l.append(mass((0.027,0,-0.067,1),1.634))#r_foot'13

#whole weight of robot
sumWeight=0
for i in range(13):
        sumWeight=sumWeight+l[i].weight
        # print sumWeight
    

massPub = rospy.Publisher("/mass",Point, queue_size=2);

if __name__ == '__main__':
    
    rospy.init_node('listener', anonymous=True)
    # timerCallback(1)
    rospy.Timer(rospy.Duration(0.02), timerCallback)

    subscriber=rospy.Subscriber("/gazebo/link_states", LinkStates, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
