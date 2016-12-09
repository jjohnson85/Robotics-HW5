

import rospy
import numpy as np
import Queue
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from PIL import Image


###path_planner.py###

    #Author Jared Johnson

def isNeighbor( worldmap, x, y ):
    for i in range( -1, 2 ):
        for j in range( -1, 2 ):
            if( worldmap[y][x] == 0 ):
                if( i == 0 and j == 0 ):
                    continue
                else:
                    if( worldmap[y+i][x+j] == 100 ):
                        return True
            
                    elif( worldmap[y+i][x+j] == -1 ):
                        return True

    return False

def brushfire( worldmap ):

    for i in range( 0, 200 ):
        for j in range( 0, 200):
            #print( worldmap[i][j] )
            if( isNeighbor(worldmap, j, i) ):

                worldmap[i][j] = 101

    for i in range( 0, 200 ):
        for j in range( 0, 200 ):
            if( worldmap[i][j] == 101 ):
                worldmap[i][j] = 100

    return worldmap

def tracepath( cX, cY, worldmap ):
    
    pmsg = Path()

    # important!
    pmsg.header.frame_id = "map"
    

    selectionList = []
    while( worldmap[cY][cX] != 1 ):
        del selectionList[:]
        bestPoint = (100000, 0, 0 )
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

            if( value[0] < bestPoint[0]):
                bestPoint = value


        worldmap[cY][cX] = -2
        i = bestPoint[1]
        j = bestPoint[2]
        cX, cY = cX+j, cY+i
        p = PoseStamped()
        p.pose.position.x = float((cX-100)/10.0)
        p.pose.position.y = float(-(cY-100)/10.0)
        pmsg.poses.append(p)
                        


    return pmsg



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
    message = msg
    

#Start a ROS node.
rospy.init_node('path_planner')

#Subscribe to the map topic
rospy.Subscriber("map", OccupancyGrid, mapcallback)

#Publish a path on the "path" topic
pub = rospy.Publisher("path", Path, queue_size=10)
rate = rospy.Rate(10) #10 hz

global message
message = 0

i = 0
while message == 0:
    i = i + 1

pmsg = Path()
worldmap = readmap( message )
worldmap = brushfire( worldmap )
worldmap = brushfire( worldmap )

worldmap = wavefront( 101, 100, 170, 80, 1, worldmap )

pmsg = tracepath( 101, 100, worldmap )
print(pmsg)

while not rospy.is_shutdown():
    
    pub.publish( pmsg )
    rate.sleep( )


