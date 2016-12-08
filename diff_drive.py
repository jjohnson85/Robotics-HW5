
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
L = 0.08
x, y, theta = 0, 0 ,0
nextX = 5
nexty = 5
timeStep = 1.0/10.0

def atPoint(inX,inY):
    return(m.sqrt((inX-x)**2+(inY-y)**2) < .1)


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
    p.velocities.append(0.0) # left wheel
    p.velocities.append(0.0) # right wheel
    cmd.points = [p]

    print( "PLZ STOP" )

    #pub.publish(cmd)
    

    
#Comute
def getTheta(x,y):
    oldTh = theta
    tr =  m.acos((x*m.cos(oldTh) + y*m.sin(oldTh))/(m.sqrt(x*x+y*y)))
    print 'theta', tr, m.pi/2
    return tr

def TurnIK( th ):
    phi2 = (th*L/r)/timeStep
    phi1 = -phi2
    return phi1, phi2

def driveIK( vel ):
    phi = vel/r
    return phi,phi

def publishDD( phi1, phi2 ):
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
    return

def DriveToPoint(inX,inY):
    #for k in range(2):
    #    theta = getTheta(inX,inY)
    #    phi1, phi2 = TurnIK(theta)
    #    publishDD( phi1, phi2 )
    #    rate.sleep()
    phi1, phi2 = driveIK(.5)
    #publishDD( phi1, phi2 )
    publishDD( 1, 1 )    
    distance = m.sqrt((inX-x)**2 + (inY-y)**2)
    cycles = int(distance/timeStep)*2
    print cycles
    for k in range(cycles):
        rate.sleep()
    phi1, phi2 = driveIK(0)
    publishDD( phi1, phi2 )
    rate.sleep()

DriveToPoint(2,0)
#DriveToPoint(2,1)
#DriveToPoint(2,-1)

