

import rospy
import numpy as np
import Queue
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
    
    selectionList = []
    while( worldmap[cY][cX] != 1 ):
        del selectionList[:]
        bestPoint = (0, 0, 0 )
        lowestPoint = (0, 0, 0)
        highestPoint = (0, 0, 0)
        currPoint = worldmap[cY][cX]

        for i in range( -1, 2 ):
            for j in range( -1, 2 ):
                if( abs(cY+i) < 200 and abs(cX+j) < 200 ):
                    point = worldmap[cY+i][cX+j]
                    if( point < worldmap[cY][cX] and point > 0 and point != 100 ):

                        selectionList.append( (point, i, j ) )

        for value in selectionList:
            if( value[0] < lowestPoint[0] ):
                lowestPoint = value
            if( value[0] > highestPoint[0] ):
                highestPoint = value

        for value in selectionList:

            if( len( selectionList) == 1 ):
                bestPoint = value

            bestPoint =value

        worldmap[cY][cX] = -2
        i = bestPoint[1]
        j = bestPoint[2]
        cX, cY = cX+j, cY+i
                        
        print cX, cY, point


    return worldmap



def wavefront( stX, stY, cX, cY, mark, worldmap ):

    WaveQueue = Queue.Queue() 
    WaveQueue.put( (cX, cY) )
    tup = (cX, cY)

    while( not WaveQueue.empty( ) ):

        cX = tup[0]
        cY = tup[1]

        for i in range( -1, 2 ):
            for j in range( -1, 2 ):
                if( not( abs( cY+i ) > 200 or abs(cX+j) > 200 )):
                    if( worldmap[cY+i][cX+j] == 0 and not (i == 0 and j == 0)):
                        WaveQueue.put( (cX+j, cY+i) )
                        worldmap[cY+i][cX+j] = mark

        tup = WaveQueue.get( )
        mark = mark + 1

        if( tup[1] == stY and tup[0] == stX ):
            worldmap[stY][stX] = mark
            break        

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
print( worldmap[169][79] )
worldmap = brushfire( worldmap )
worldmap = brushfire( worldmap )
#worldmap = brushfire( worldmap )

#for z in range(0, 200):
 #   for j in range(0, 200):
  #      if( worldmap[z][j] == 0 ):
   #        worldmap[z][j] = 300

worldmap = wavefront( 99, 99, 169, 79, 1, worldmap )


print( worldmap[79][169] )
worldmap = tracepath( 99, 99, worldmap )
img = Image.fromarray( worldmap )

img.show( )

while not rospy.is_shutdown():
    i = i + 1
