import math as m
import rospy
from geometry_msgs.msg import Pose2D
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from botClasses import *
import time
import cubic_spline as cs

currPose = Pose()

r = 0.5
L = .15/2+.02/2
speed = 1
def getTheta(x,y):
    oldTh = currPose.th
    return m.acos((x*m.cos(oldTh) + y*m.sin(oldTh))/(m.sqrt(x*x+y*y)))

def callbackGPS(msg):
    currPose.x = msg.x
    currPose.y = msg.y
    currPose.th = msg.theta

def atPoint(inX,inY):
    return(abs(inX-currPose.x) < 0.1 and abs(inY-currPose.y) < 0.1)

def drive(vel):
    phi = vel
    return phi,phi

def turn(theta,time):
    print theta
    phi1 = theta*L/r/time
    phi2 = -phi1
    return phi1,phi2

def publishVelocities(phi1,phi2):
     # Create a message to publish
    cmd = JointTrajectory()

    # Add joint names for our left and right wheels
    cmd.joint_names.append("left")
    cmd.joint_names.append("right")

    # Add our wheel velocities (radians/sec)
    p = JointTrajectoryPoint()
    p.velocities.append(phi1) # left wheel
    p.velocities.append(phi2) # right wheel
    cmd.points = [p]
    # Publish our wheel velocities
    pub.publish(cmd)

def driveToPoint(newX,newY):
    

    dotphi1, dotphi2 =  cs.GetTheta(currPose.x,currPose.y,newX,newY,speed*m.cos(currPose.th),speed*m.sin(currPose.th),speed)

    print currPose.x,currPose.y,newX,newY,speed*m.cos(currPose.th),speed*m.sin(currPose.th),speed
    phi1 = 0
    phi2 = 0

    for k in range(len(dotphi1)):
        phi1 = dotphi1[k]
        phi2 = dotphi2[k]
        publishVelocities(phi1,phi2)
        time.sleep(0.1)
        #rate.sleep()


rospy.init_node('drive_robot')
rospy.Subscriber("gps", Pose2D, callbackGPS)
pub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)
rate = rospy.Rate(10) # 10 hz

driveToPoint(3.0,2.0)
#driveToPoint(2.0,1.0)
publishVelocities(0,0)
#while not rospy.is_shutdown():
    #time.sleep(1)
    #driveToPoint(1,0)
    #rate.sleep()
    #print currPose.x, currPose.y, currPose.th
    


