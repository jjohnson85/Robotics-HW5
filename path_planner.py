

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
from PIL import Image

def isNeighbor( worldmap, x, y ):
    for i in range( -1, 1 ):
        for j in range( -1, 1 ):
            if( i == 0 and j == 0 ):
                break
            else:
                if( worldmap[y+i][x+j] == 100 ):
                    return True

    return False

def brushfire( worldmap ):

    for i in range( 0, 200 ):
        for j in range( 0, 200):
            if( isNeighbor(worldmap, j, i) and worldmap[i][j] != 100 ):
                worldmap[i][j] = 101

    for i in range( 0, 200 ):
        for j in range( 0, 200 ):
            if( worldmap[i][j] == 101 ):
                worldmap[i][j] = 100

    return worldmap

def tracepath( cX, cY, worldmap ):

    for i in range( -1, 1 ):
        for j in range( -1, 1 ):
            if( worldmap[cY+i][cX+j] < worldmap[cY][cX] ): 

                worldmap[cY][cX] = -1
                tracepath( cX+j, cY+i, worldmap )
                

    return worldmap



def wavefront( stX, stY, cX, cY, mark, worldmap ):

    if( cY == stY and cX == stX ):
        worldmap[cY][cX] = mark
        print( "Hello World" )
        return worldmap

    if( worldmap[cY][cX] == 0 ):
        #print( "Je;" )
        worldmap[cY][cX] = mark

    for i in range( -1, 1 ):
        for j in range( -1, 1 ):
            if( not( abs( cY+i ) > 200 or abs(cX+j) > 200 )):

                if( worldmap[cY+i][cX+j] == 0 and not (i == 0 and j == 0)):
                    worldmap = wavefront( stX, stY, cX+j, cY+i, mark+1, worldmap ) 

    #print( worldmap[cY][cX] , " " , cY, " ", cX)
    return worldmap

def readmap( msg ):

    worldmap = np.zeros( (200,200) )

    for i in range( 0 , 200 ):
        for j in range( 0 , 200):
                #To convert from internal map point to Occupancy Grid map point
                #subtract 100 and divide by 10.0
                x = (j - 100)/10.0
                y = -(i - 100)/10.0
                xindex = int((x-msg.info.origin.position.x)*(1.0/msg.info.resolution))
                yindex = int((y-msg.info.origin.position.y)*(1.0/msg.info.resolution))
                dataindex = yindex*msg.info.width + xindex

                worldmap[i][j] = int(msg.data[dataindex])

    return worldmap


def mapcallback(msg):
    global message
    print "----- New Map -----"
    print "Width: ", msg.info.width
    print "Height: ", msg.info.height
    print "Resolution: ", msg.info.resolution
    print "Origin: \n", msg.info.origin

    message = msg
    

# Start a ROS node.
rospy.init_node('map_read_test')
# Subscribe to the map topic
rospy.Subscriber("map", OccupancyGrid, mapcallback)

global message
message = 0

i = 0
while message == 0:
    i = i + 1

worldmap = readmap( message )
print( worldmap[170][120] )
worldmap = brushfire( worldmap )
worldmap = brushfire( worldmap )
#worldmap = brushfire( worldmap )


worldmap = wavefront( 100, 100, 170, 120, 1, worldmap )
print( worldmap[169][119] )
worldmap = tracepath( 170, 120, worldmap )
img = Image.fromarray( worldmap )

img.show( )



while not rospy.is_shutdown():
    i = i + 1
