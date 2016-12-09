
import rospy
import math as m
import time
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Path


### diff_drive.py 

    #Auhtor Marcus Haberling

def callbackGPS(msg):
    global x, y ,theta  
    x = msg.x
    y = msg.y
    theta = msg.theta

def callbackPath(msg):
    global path
    path = msg.poses

# Start a ROS node.
rospy.init_node('drive_robot_test')
# Wheel velocity publisher
pub = rospy.Publisher("cmd_joint_traj", JointTrajectory, queue_size=10)
rospy.Subscriber("gps", Pose2D, callbackGPS)
rospy.Subscriber("path", Path, callbackPath)
# Rate to publish the wheel velocities
rate = rospy.Rate(10) # 10 hz
global path
path = 0


r = .05
L = 0.15
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
def getTheta(inX,inY):
    oldTh = theta
    tr =  m.acos((inX*m.cos(oldTh) + inY*m.sin(oldTh))/(m.sqrt(inX*inX+inY*inY)))
    #print 'theta', tr, m.pi/2

    #return tr
    print m.atan2(inY-y, inX-x)
    
    tr = m.atan2(inY-y, inX-x)
    if tr > m.pi:
        return tr - m.pi*2
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
    global x,y,theta
    diffTheta = m.atan2(inY-y,inX-x) - theta
    print( "DIFF: " , diffTheta )
    if( diffTheta > 0.0 ):
        print( "Publish" )
        publishDD(-0.4, .4)
    elif( diffTheta < 0.0 ):
        publishDD(.4,-0.4)
    while abs(diffTheta) > .1:
        diffTheta = m.atan2(inY-y,inX-x) - theta
        #phi1, phi2 = TurnIK(theta)
        #publishDD( phi1, phi2 )
        #rate.sleep()

    phi1, phi2 = driveIK(.5)
    publishDD( phi1, phi2 )

    #publishDD( 1, 1 )    
    distance = m.sqrt((inX-x)**2 + (inY-y)**2)
    cycles = int(distance/timeStep)*2

    for k in range(cycles):
        rate.sleep()
    phi1, phi2 = driveIK(0)
    publishDD( phi1, phi2 )
    rate.sleep()
i=0
while(path == 0 ):
    i = i+1

i = 1
for pose in path:
    dx = pose.pose.position.x 
    dy = pose.pose.position.y
    print( "PIN T " , dx, " ",  dy) 
    DriveToPoint( dx, dy)   


#DriveToPoint(2,0.5)
#DriveToPoint(2,1)
#DriveToPoint(2,-1)
publishDD( 0, 0 )
