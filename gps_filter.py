import rospy
from geometry_msgs.msg import Pose2D
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import numpy as np
import tf
from geometry_msgs.msg import PoseStamped

### gps_filter

    #Author Jared Johnson

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
pubrviz = rospy.Publisher("pose_filtered", PoseStamped, queue_size = 10 )
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
msg = PoseStamped( )
msg.header.frame_id = "map"

W = np.array([[0.2,0,0],[0,0.2,0],[0,0,0.2]])

V = np.array([[0.001,0,0],[0,0.001,0],[0,0,0.001]])
P = np.array([[0,0,0],[0,0,0],[0,0,0]])

H = np.array([[1, 0, 0],[0, 1, 0],[0, 0, 1]])

xk = 0.0
yk = 0.0
tk = 0.0

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
    print( xhat, phi1, phi2 )
    #Predictive covariance
    P = Fk.dot(P.dot(FkT) ) + V
    print( P )

    #Kalman Gain
    K = P.dot( np.linalg.inv( P + W ) )
    print( K )

    #Fetch observation
    xk = posemessage.x
    yk = posemessage.y
    tk = posemessage.theta

    z = np.array([[xk],[yk],[tk]])

    print( xhat )
    xhat = xhat + K.dot(z - xhat)
    print( xhat )
    P = (H - K).dot(P)

    #Update x,y,theta values with pose estimate and publish them
    pose.x = xk = xhat[0][0]
    pose.y = yk = xhat[1][0]
    pose.theta = tk = xhat[2][0]
    print(pose)

    # Publish Position (1,0)
    msg.pose.position.x = xk
    msg.pose.position.y = yk
    


    # Publish theta = pi/4
    # We need to convert from euler coordinates to a quaternion.
    quaternion = tf.transformations.quaternion_from_euler(0,0,tk)
    msg.pose.orientation.x = quaternion[0]
    msg.pose.orientation.y = quaternion[1]
    msg.pose.orientation.z = quaternion[2]
    msg.pose.orientation.w = quaternion[3]

    pubrviz.publish( msg )
    pub.publish(pose)
    #exit( )
    rate.sleep( )

    









