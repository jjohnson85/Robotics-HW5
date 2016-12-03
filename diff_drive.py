
import rospy
import math as m
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose2D

def callbackGPS(msg):
    global x, y ,theta  
    x = msg.x
    y = msg.y
    theta = msg.theta

# Start a ROS node.
rospy.init_node('drive_robot_test')
# Wheel velocity publisher
pub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)
rospy.Subscriber("gps", Pose2D, callbackGPS)
# Rate to publish the wheel velocities
rate = rospy.Rate(10) # 10 hz



r = .05
L = 0.15
x, y, theta = 0, 0 ,0
nextX = 5
nexty = 5


def atLoc(inX, inY):
    return( abs(x - inX) < 1 and abs(y - inY) < 1 )

def IK( inX, inY ):
    if atLoc(inX, inY):
        return
    newTh = m.atan2(inX,inY) - theta
    phi1, phi2 = TurnIK(newTh)

    # Create a message to publish
    cmd = JointTrajectory()

    # Add joint names for our left and right wheels
    cmd.joint_names.append("left")
    cmd.joint_names.append("right")

    # Add our wheel velocities (radians/sec)
    p = JointTrajectoryPoint()
    p.velocities.append( phi1 ) # left wheel
    p.velocities.append( phi2 ) # right wheel
    cmd.points = [p]

    # Publish our wheel velocities
    pub.publish(cmd)
    
    time.sleep(100)

    p = JointTrajectoryPoint()
    p.velocities.append( 0.0 ) # left wheel
    p.velocities.append( 0.0 ) # right wheel
    cmd.points = [p]

    print( "PLZ STOP" )

    #pub.publish(cmd)
    

    
#Comute


def TurnIK( th ):
    phi1 = th*L/r
    phi2 = -phi1
    return phi1, phi2

def driveIK( vel ):
    phi = vel/r
    return phi,phi

def publishDD( phi, phi2 ):
 #Publish wheel velocities for the DD robot
    return

IK( 5, 5 )


