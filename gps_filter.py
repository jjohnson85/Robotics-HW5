import rospy
from geometry_msgs.msg import Pose2D
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np

# Documentation for this message type:
# http://docs.ros.org/kinetic/api/geometry_msgs/html/msg/Pose2D.html
def posecallback(msg):
    global posemessage
    posemessage = msg

def velcallback(msg):
    global velmessage
    velmessage = msg

rospy.init_node('gps_filter')
rospy.Subscriber("gps", Pose2D, posecallback)
rospy.Subscriber("cmd_joint_traj", JointTrajectory, velcallback ) 
pub = rospy.Publisher("filtered_gps", Pose2D, queue_size = 10 )
rate = rospy.Rate(10)

global posemessage, velmessage
posemessage = 0
velmessage = 0

i = 0
#Wait for gps
while( posemessage == 0 or velmessage == 0 ):
    i = i + 1

#Initial values
pose = Pose2D( )

V = np.array([[0.2,0,0],[0,0.2,0],[0,0,0.2]])

W = P = np.array([[0,0,0],[0,0,0],[0,0,0]])

H = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])

xk = 0.0
yk = 0.0
tk = posemessage.theta

r = 0.05
L = 0.15
dt = 0.1

# Loop until we shut the node down (control-c).
while not rospy.is_shutdown():
    #Run the EKF and publish the filtered gps data 
    phi1 = velmessage.points[0].velocities[0]
    phi2 = velmessage.points[0].velocities[1]
    
    xupdate = xk + r*dt/2.0 * (phi1 + phi2) * np.cos(tk)
    yupdate = yk + r*dt/2.0 * (phi1 + phi2) * np.sin(tk)
    tupdate = tk + r*dt/2.0*L * (phi1 - phi2)
    
    Fval1 = -r*dt/2.0 * (phi1 + phi2) * np.sin(tk)
    Fval2 = r*dt/2.0 * (phi1 + phi2) * np.cos(tk)
    Fk = np.array([[1, 0, Fval1],[0,1,Fval2],[0,0,1]])
    FkT = Fk.T

    #Predictive update given control input
    xhat = np.array([[xupdate],[yupdate],[tupdate]]) 

    #Predictive covariance
    P = Fk.dot(P.dot(FkT)) + V

    #Kalman Gain
    K = P.dot( np.linalg.inv(P) )

    #Fetch observation
    xk = posemessage.x
    yk = posemessage.y
    tk = posemessage.theta

    z = np.array([[xk],[yk],[tk]])
    print( xhat )
    xhat = xhat + K.dot(z - xhat)

    P = (H - K).dot(P)

    #Update x,y,theta values with pose estimate and publish them
    pose.x = xk = xhat[0][0]
    pose.y = yk = xhat[1][0]
    pose.theta = tk = xhat[2][0]
    print(pose)
    pub.publish(pose)
    rate.sleep( )

    









