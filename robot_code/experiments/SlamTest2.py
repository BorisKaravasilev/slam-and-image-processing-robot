#!/usr/bin/python3
from cmath import pi
from logging import exception
from multiprocessing.resource_sharer import stop
import os
import numpy as np
import math
import traceback
import pickle

# SLAM imports
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import RPLidarA1 as LaserModel
from rplidar import RPLidar as Lidar

# Thymio imports

# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import matplotlib.pyplot as plt
from time import sleep
import dbus
import dbus.mainloop.glib
from threading import Thread

# SLAM constants
MAP_SIZE_PIXELS         = 250
MAP_SIZE_METERS         = 15
LIDAR_DEVICE            = '/dev/ttyUSB0'

MAP_SIZE_MM = MAP_SIZE_METERS * 1000
TILE_SIZE_MM = MAP_SIZE_MM / MAP_SIZE_PIXELS

# Pose will be modified in our threaded code
pose = [0, 0, 0]

#Goals
goalX = 122
goalY = 125

# Ideally we could use all 250 or so samples that the RPLidar delivers in one
# scan, but on slower computers you'll get an empty map and unchanging position
# at that rate.
MIN_SAMPLES   = 50

# ======================================= Our functions

def getPosition():
    return pose

def getMap():
    return mapbytes

def getGridPosition(pose):
    """
    Converts the robots position from milimeters to grid tile coordinates.
    """

    x = pose[0]
    y = pose[1]

    return x // TILE_SIZE_MM, y // TILE_SIZE_MM

def printMap(map):
    for y in range(MAP_SIZE_PIXELS):
        for x in range(MAP_SIZE_PIXELS):
            map_char = ' '

            if map[x + y * MAP_SIZE_PIXELS] < 127:
                map_char = '.'
            elif map[x + y * MAP_SIZE_PIXELS] > 127:
                map_char = '*'
            
            print(f"{map_char}, ", end="")
        
        print()

def calcTriangle(targetX, targetY):
        grid_x, grid_y = getGridPosition(getPosition())

        # get a and b sides of the triangle
        
        # a = abs(targetX - grid_x)
        # b = abs(targetY - grid_y)
        a = targetX - grid_x
        b = targetY - grid_y
       
        #calculate c side of the triangle

        c = math.sqrt(a**2 + b**2)
       


        return a, b, c

def transfromTheta():
        theta = pose[2]

        #position of the robot between 0 and 360
        thetaDeg = 0

        #calculate the position of the robot between 0 and 360
        thetaDeg = theta % 360

        if thetaDeg < 0:
            thetaDeg = 360 + thetaDeg
        return thetaDeg

def calcRotDeg():
    a, b, c = calcTriangle(goalX, goalY)    
    grid_x, grid_y = getGridPosition(getPosition())
    
    if a == 0 and b > 0:
        targetDeg = 90
    elif a ==0 and b < 0:
        targetDeg = 270
    else:
        targetDeg = math.degrees(math.atan(b/a))

    if grid_x > goalX and grid_y < goalY:
        targetDeg = targetDeg - 180
    
    if grid_x > goalX and grid_y > goalY:
        targetDeg = targetDeg + 180


    rotDeg = 0
    rotDeg = targetDeg - rotDeg

    if rotDeg == -0.0:
        rotDeg = 180

    if rotDeg < 0:
        rotDeg = 360 + rotDeg
    
    return rotDeg

# def calcRotDeg2():
#     a, b, c = calcTriangle(goalX, goalY)    
#     grid_x, grid_y = getGridPosition(getPosition())
#     targetDeg = math.degrees(math.atan(b/a))
#     if (goalX > grid_x and goalY > grid_y):
#         rotdeg = targetDeg - rotDeg


#     rotDeg = 0
#     rotDeg = targetDeg - rotDeg

#     if rotDeg < 0:
#         rotDeg = 360 + rotDeg
#     #return rotDeg
#     return rotDeg


class Thymio:
    def __init__(self):
        self.aseba = self.setup()
        # self.sens1Table = [0, 0, 0, 0, 0, 0, 0, 0]
        # self.sens2Table = [0, 0, 1680, 2016, 2261, 2531, 2870, 3354]
        self.sensTable = [1775, 2027, 2283, 2517, 2824, 3275, 3755, 4316]
        # self.sens4Table = [0, 0, 1454, 1758, 2095, 2496, 2902, 3456]
        # self.sens5Table = [0, 0, 0, 0, 0, 0, 0, 0]
        # self.sensorsTable  = [self.sens1Table, self.sens2Table, self.sens3Table, self.sens4Table, self.sens5Table]

        self.distanceSensors = [10, 9, 8, 7, 6, 5, 4, 3]

    def drive(self, left_wheel_speed, right_wheel_speed):
        print("Left_wheel_speed: " + str(left_wheel_speed))
        print("Right_wheel_speed: " + str(right_wheel_speed))
        
        left_wheel = left_wheel_speed
        right_wheel = right_wheel_speed

        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def drive2cm(self, velocityL, velocityR):
        count = 0
        while count < 200:
            self.drive(velocityL, velocityR)
            count += 1

    def stop(self):
        left_wheel = 0
        right_wheel = 0
        self.aseba.SendEventName("motor.target", [left_wheel, right_wheel])

    def sens(self):

        prox_horizontal = self.aseba.GetVariable("thymio-II", "prox.horizontal")
        print("Sensing:")
        print(prox_horizontal[0]) #RIGHTMOST when looking at robot front
        print(prox_horizontal[1])
        print(prox_horizontal[2])
        print(prox_horizontal[3])
        print(prox_horizontal[4])
        return [prox_horizontal[0],prox_horizontal[1], prox_horizontal[2], prox_horizontal[3], prox_horizontal[4]]

    def inRangeOfWall(self, sensors):
        inRange = False
        for sensor in sensors:
            if sensor > 50: 
                inRange = True
                # Print the index of the sensor that detected the wall
                print(sensors.index(sensor))
        return inRange

    # def valuesToDistances(self, recordedValues):
    #     distances = []
    #     for sensor, sensorListOfValues in zip(recordedValues, self.sensorsTable):
    #         closest = min(sensorListOfValues, key=lambda x: abs(x-sensor))
    #         idx_value = sensorListOfValues.index(closest)
    #         distances.append(self.distancesSensors[idx_value])

    #     return distances


    def sensValuesToDistance(self, sensorsValues):
        distances = []
        for sensor in sensorsValues:
            if sensor <= self.sensTable[0]:
                distances.append(0)#self.distanceSensors[0])
            elif sensor >= self.sensTable[-1]:
                distances.append(self.distanceSensors[-1])
            else:
                closestIdx = self.sensTable.index(min(self.sensTable, key=lambda x: abs(x-sensor)))
                distances.append(self.distanceSensors[closestIdx])
        return distances

    def rotateToTarget(self):
    
        rotDeg = calcRotDeg()
        thetaDeg = transfromTheta()
        diff = abs(rotDeg - thetaDeg)

    
        print('WE ARE IN THE ROTATEDEG FUNCTION YAY')
        #start turning right if rotDeg < 180 degrees
        if  rotDeg < 180:
            if rotDeg >= thetaDeg:
                self.drive(100, -100)
            #correction
            elif rotDeg <= thetaDeg:
                self.drive(-100, 100)
            else:
                self.stop()
        #start turning left if rotDeg > 180 degrees
        elif rotDeg > 180:
            if rotDeg <= thetaDeg:
                self.drive(-100, 100)
            #correction
            elif rotDeg >= thetaDeg:
                self.drive(100, -100)
            else:
                self.stop()
        #stop the robot if we are on target (more or less)
        elif diff > 5 or diff <5:
            self.stop()
    

  

    def driveToTarget(self, targetX, targetY):
        grid_x, grid_y = getGridPosition(getPosition())
        rotDeg = calcRotDeg()

        #drive forward if robot is not on target and is facing the target
        while grid_x != targetX and grid_y != targetY and rotDeg == 0:
            self.drive(100, 100)
        # corrigate if robot is still not on target or is not facing the target
        if (grid_x != targetX or grid_y !=targetY) or rotDeg != 0:
            self.rotateToTarget()
            self.driveToTarget()

        self.stop()

    ############## Bus and aseba setup ######################################

    def setup(self):
        print("Setting up")
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        bus = dbus.SessionBus()
        asebaNetworkObject = bus.get_object('ch.epfl.mobots.Aseba', '/')

        asebaNetwork = dbus.Interface(
            asebaNetworkObject, dbus_interface='ch.epfl.mobots.AsebaNetwork'
        )
        # load the file which is run on the thymio
        asebaNetwork.LoadScripts(
            'thympi.aesl', reply_handler=self.dbusError, error_handler=self.dbusError
        )

        # scanning_thread = Process(target=robot.drive, args=(200,200,))
        return asebaNetwork

    def stopAsebamedulla(self):
        os.system("pkill -n asebamedulla")

    def dbusReply(self):
        # dbus replys can be handled here.
        # Currently ignoring
        pass

    def dbusError(self, e):
        # dbus errors can be handled here.
        # Currently only the error is logged. Maybe interrupt the mainloop here
        print("dbus error: %s" % str(e))


# ----------- Our Functions
base_velocity = 160

def get_wheel_velocities(distances):
    """
    Simple obstacle avoiding controller that is biased to steer left.
    """

    l2, l1, m, r1, r2 = distances

    correction = l2 + 2*l1 + 3.5*m - 2*r1 - r2
    correction = correction * 30
    left_wheel_velocity = base_velocity + correction
    right_wheel_velocity = base_velocity - correction
    return left_wheel_velocity, right_wheel_velocity

def main():
    global pose
    robot = Thymio()

    # SLAM

    # Connect to Lidar unit
    lidar = Lidar(LIDAR_DEVICE)

    # Create an RMHC SLAM object with a laser model and optional robot model
    slam = RMHC_SLAM(LaserModel(), MAP_SIZE_PIXELS, MAP_SIZE_METERS)


    # Initialize an empty trajectory
    trajectory = []

    # Initialize empty map
    mapbytes = bytearray(MAP_SIZE_PIXELS * MAP_SIZE_PIXELS)

    # Create an iterator to collect scan data from the RPLidar
    iterator = lidar.iter_scans()

    # We will use these to store previous scan in case current scan is inadequate
    previous_distances = None
    previous_angles    = None

    # First scan is crap, so ignore it
    next(iterator)
    pose = [0, 0, 0]

    

    while True:

        # Extract (quality, angle, distance) triples from current scan
        # print(next(iterator))
        items = [item for item in next(iterator)]

        # Extract distances and angles from triples
        distances = [item[2] for item in items]
        angles    = [item[1] for item in items]

        # Update SLAM with current Lidar scan and scan angles if adequate
        if len(distances) > MIN_SAMPLES:
            slam.update(distances, scan_angles_degrees=angles)
            previous_distances = distances.copy()
            previous_angles    = angles.copy()

        # If not adequate, use previous
        elif previous_distances is not None:
            slam.update(previous_distances, scan_angles_degrees=previous_angles)

        # Get current robot position
        pose[0], pose[1], pose[2] = slam.getpos()

        # Get current map bytes as grayscale
        slam.getmap(mapbytes)

        ## Serializes the map object to a file
        # outfile = open("mapbytes",'wb')
        # pickle.dump(mapbytes, outfile)
        # outfile.close()

        # print(f"x:{pose[0]} y:{pose[1]} theta {pose[2]}")
        

        try:   
            print("getCoordinates(): " + f"x:{getPosition()[0]} y:{getPosition()[1]} theta:{getPosition()[2]} thetaDeg:{transfromTheta()} rotDeg:{calcRotDeg()}")
            grid_x, grid_y = getGridPosition(getPosition())
            print(f"grid_x = {grid_x} grid_y = {grid_y}")

            #printMap(mapbytes)
            robot.rotateToTarget()
            
            #robot.drive(-100, 100)
            #robot.driveToTarget(goalX, goalY)
            #robot.stop()
        except Exception as e:
            print (e)

        
        #robot.drive(-100, 100)
        

 
 




    lidar.stop()
    lidar.disconnect()


    # for i in range(10000):
    #     sensors = robot.sens()
    #     # print("Sensor values:\n", sensors)

    #     distances = robot.sensValuesToDistance(sensors)
    #     print("Distances: ", distances)

    #     left_wheel_velocity, right_wheel_velocity = get_wheel_velocities(distances)
    #     robot.drive(left_wheel_velocity, right_wheel_velocity)

    
    


    robot.stop()


# ----------------- loop end ---------------------


if __name__ == '__main__':
    try:
        main()
        os.system("pkill -n asebamedulla")
    except Exception as e:
        print("Stopping robot with exception:\n\n" + str(e))
        exit_now = True
        sleep(5)
        os.system("pkill -n asebamedulla")
        # print("asebamodulla killed")