

import rospy
from nav_msgs.msg import OccupancyGrid
    

def brushfire( worldmap ):

    return



def wavefront( stX, stY, cX, cY, worldmap ):

    return

def readmap( msg ):

    worldmap = []

    #Convert our map into a discrete 800*800 map for better resolution
    #for our path planning as well as easy indexing for later programming
    for i in range( 0 , msg.info.width*4 -1):
        for j in range( 0 , msg.info.height*4 -1):
                x = i/4.0
                y = j/4.0
                xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
                yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
                dataindex = yindex*msg.info.width + xindex
                worldmap[i][j] = msg.data[dataindex]

    print( worldmap )
    return worldmap


def mapcallback(msg):
    print "----- New Map -----"
    print "Width: ", msg.info.width
    print "Height: ", msg.info.height
    print "Resolution: ", msg.info.resolution
    print "Origin: \n", msg.info.origin

    worldmap = readmap( msg )
    print(worldmap)
    

# Start a ROS node.
rospy.init_node('map_read_test')
# Subscribe to the map topic
rospy.Subscriber("map", OccupancyGrid, mapcallback)

