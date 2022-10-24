#!/usr/bin/python3
from cmath import pi
from logging import exception
from multiprocessing.resource_sharer import stop
import os
import numpy as np
import math
import traceback
import pickle
import cv2 as cv
import traceback

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
goalX = 115
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

    return int(x // TILE_SIZE_MM), int(y // TILE_SIZE_MM)

def printMap(map):
    for y in range(MAP_SIZE_PIXELS):
        for x in range(MAP_SIZE_PIXELS):
            map_char = ' '

            if map[x + y * MAP_SIZE_PIXELS] < 127:
                map_char = '.'
            elif map[x + y * MAP_SIZE_PIXELS] > 127:
                map_char = '*'
            
            # print(f"{map_char}, ", end="")
            print(map[x + y * MAP_SIZE_PIXELS])
        
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
    print(f"a: {a}, b: {b}, c: {c}")
    
    if a == 0 and b > 0:
        targetDeg = 90
    elif a == 0 and b < 0:
        targetDeg = 270
    elif not a == 0:
        targetDeg = math.degrees(math.atan(b/a))
    else:
        targetDeg = 0

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



class Thymio:
    def __init__(self):
        self.aseba = self.setup()
        self.sensTable = [1775, 2027, 2283, 2517, 2824, 3275, 3755, 4316]
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
        # print("Sensing:")
        # print(prox_horizontal[0]) #RIGHTMOST when looking at robot front
        # print(prox_horizontal[1])
        # print(prox_horizontal[2])
        # print(prox_horizontal[3])
        # print(prox_horizontal[4])
        return [prox_horizontal[0],prox_horizontal[1], prox_horizontal[2], prox_horizontal[3], prox_horizontal[4]]

    def inRangeOfWall(self, sensors):
        inRange = False
        for sensor in sensors:
            if sensor > 50: 
                inRange = True
                # Print the index of the sensor that detected the wall
                print(sensors.index(sensor))
        return inRange


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

    def needs_rotation(self, angle_error_tolerance = 10):
        rotDeg = calcRotDeg()
        thetaDeg = transfromTheta()
        diff = abs(rotDeg - thetaDeg)
        print(f"diff: {diff}, offset: {angle_error_tolerance}")
        print(diff > angle_error_tolerance)
        return diff > angle_error_tolerance
        

    def correct_rotation(self):
        rotDeg = calcRotDeg()
        thetaDeg = transfromTheta()
        diff = abs(rotDeg - thetaDeg)

        #start turning right if rotDeg < 180 degrees
    
        if  rotDeg < 180:
            print('rotDeg < 180')
            if rotDeg >= thetaDeg:
                self.drive(100, -100)
            #correction
            elif rotDeg <= thetaDeg:
                self.drive(-100, 100)

        #start turning left if rotDeg > 180 degrees
        elif rotDeg > 180:
            print('rotDeg > 180')
            if rotDeg <= thetaDeg:
                self.drive(-100, 100)
            #correction
            elif rotDeg >= thetaDeg:
                self.drive(100, -100)
  

    def driveToTarget(self, targetX, targetY):
        if self.needs_rotation():
            print("rotating")
            self.correct_rotation()
        else:
            print("driving straight")
            self.drive(400, 400)


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


# =============================== Tennis Ball Detection

class TennisBallDetector:
    def __init__(self, target_hsv=(34, 132, 206), threshold_range=(4, 20, 100), min_contour_area=250, ui=False):
        print("start")
        # Detection Settings
        self.hsv_target = target_hsv
        self.hsv_range = threshold_range
        self.hsv_min, self.hsv_max = self.get_boundaries(self.hsv_target, self.hsv_range)
        self.min_contour_area = min_contour_area
        self.ui = ui

        # Video Capture Settings
        self.video_capture_device_index = 0
        self.video = cv.VideoCapture(self.video_capture_device_index)
        print("vid")

        # UI
        self.key = None

        if self.ui:
            cv.namedWindow("Original", cv.WINDOW_AUTOSIZE)
            cv.namedWindow("Before", cv.WINDOW_AUTOSIZE)
            cv.namedWindow("After", cv.WINDOW_AUTOSIZE)

            cv.moveWindow("Original", 1300, 30)
            cv.moveWindow("Before", 50, 30)
            cv.moveWindow("After", 700, 30)

        print("Detector initialized...")

    def get_last_frame(self):
        _, last_frame = self.video.read()
        return last_frame

    def get_boundaries(self, hsv_target, hsv_range):
        lower_boundary = np.array(hsv_target) - np.array(hsv_range)
        upper_boundary = np.array(hsv_target) + np.array(hsv_range)
        return lower_boundary, upper_boundary

    def threshold_img_hsv(self, hsv_img):
        # Find the colors within the boundaries
        binary_img = cv.inRange(hsv_img, self.hsv_min, self.hsv_max)
        return binary_img

    def detect(self):
        img = self.get_last_frame()

        if self.ui:
            cv.imshow("Original", img)

        # Blur
        blurred_img = cv.GaussianBlur(img, (7, 7), 0)

        # Convert to HSV color space
        hsv_img = cv.cvtColor(blurred_img, cv.COLOR_BGR2HSV)

        # Color segmentation
        binary_img = self.threshold_img_hsv(hsv_img)

        if self.ui:
            cv.imshow("Before", binary_img)

        # Morphological operations
        ellipse_kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))
        morph_img = cv.morphologyEx(binary_img, cv.MORPH_OPEN, ellipse_kernel)
        morph_img = cv.morphologyEx(morph_img, cv.MORPH_CLOSE, ellipse_kernel)

        # Find the contour with the biggest area
        contours, hierarchy = cv.findContours(morph_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        bgr_img = cv.cvtColor(morph_img, cv.COLOR_GRAY2BGR)

        biggest_contour_area = 0

        if len(contours) > 0:
            biggest_contour = max(contours, key=cv.contourArea)
            biggest_contour_area = cv.contourArea(biggest_contour)
            line_color = (0, 0, 255)
            cv.drawContours(bgr_img, biggest_contour, -1, line_color, 3)  # -1 = all contours

        if self.ui:
            cv.imshow("After", bgr_img)
            self.key = cv.waitKey(1)

        return biggest_contour_area, biggest_contour_area > self.min_contour_area



# ----------- Our Functions
base_velocity = 160

def get_wheel_velocities(distances):
    """
    Simple obstacle avoiding controller that is biased to steer left.
    """

    l2, l1, m, r1, r2 = distances

    correction = l2 + 2*l1 + 3.5*m - 2*r1 - r2
    correction = correction * 36
    left_wheel_velocity = base_velocity + correction
    right_wheel_velocity = base_velocity - correction
    return left_wheel_velocity, right_wheel_velocity

def show_map_in_window(mapbytes, robot_x, robot_y, target_x, target_y, angle):
    map_img = np.array(mapbytes).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    bgr_map_img = cv.cvtColor(map_img, cv.COLOR_GRAY2BGR)
    bgr_map_img = cv.ellipse(bgr_map_img, (robot_x, robot_y), (4, 2), angle, 0, 360, (0, 0, 255), 2)
    bgr_map_img = cv.ellipse(bgr_map_img, (target_x, target_y), (1, 1), 0, 0, 360, (255, 0, 0), 2)
    cv.namedWindow("Map", cv.WINDOW_AUTOSIZE)
    cv.imshow("Map", bgr_map_img)
    cv.waitKey(1)

# Given a flatten (1D list) of the map, 
# returns a 2D version of it where:
# map[Y][X] == map[row][column]
def mapTo2D(map, division=MAP_SIZE_PIXELS):
    return np.array_split(np.array(map), division)

# Scan in the 8 directions surrounding the robot.
# Returns the sum of the probabilities values within
# the specified radius of the robot.
def scan_directions(x, y, myMap, radius, directions_coords):
    counters = dict.fromkeys(directions_coords, 0)
    for dx, dy in counters:
        try:
            for i in range(1, radius+1):
                if y+dy*i < 0 or x+dx*i < 0:
                    counters[(dx,dy)] += 255
                else:
                    counters[(dx,dy)] += myMap[y+dy*i][x+dx*i]
        except IndexError:
            pass
    return list(counters.values())




def isWall(value):
    return True if value < 30 else False

# Given a min distance, checks if the robot is in range of a wall.
def isWallPresent(x, y, coordinates, min_wall_distance, map):
    dx, dy = coordinates[0], coordinates[1]
    wallPresent = False
    try:
        for i in range(1, min_wall_distance+1):
            if y+dy*i <0 or x+dx*i < 0:
                wallPresent = True
                break
            currentAnalyzedTile = map[y+dy*i][x+dx*i]
            if isWall(currentAnalyzedTile):
                wallPresent = True
    except IndexError:
        wallPresent = True
    return wallPresent

# Checks and each direction around the robot and generates
# a list of 0s and 1s: wall in the direction --> 0 else 1
def get_legal_directions(x, y, map, min_wall_distance, directions_coords):
    newMap= list.copy(map)
    newMap.reverse()
    directions = [1,1,1,1,1,1,1,1]  # [T, TL, L, BL, B, BR, R, TR]
    for idx, directionCoord in enumerate(directions_coords):
        if isWallPresent(x,y, directionCoord, min_wall_distance, newMap):
            directions[idx] = 0
    return directions

# Returns the new position to be used for the robot based on
# which direction has the least amount of wall "presence".
def next_tile(x,y, scanned_directions, legal_directions, directions_coords):
    idx = scanned_directions.index(max(scanned_directions))
    if legal_directions[idx] == 1 or scanned_directions[idx] == -1000000:
        direction = directions_coords[idx]
        return (int(x + direction[0]), int(y + direction[1]))
    else:
        dirsCopy = list.copy(scanned_directions)
        dirsCopy[idx] = -1000000
        direction = next_tile(x, y, dirsCopy, legal_directions, directions_coords)
        return (direction)
    
def get_direction_visualisation_coords(grid_x, grid_y, next_x, next_y, offset = 10):
    return grid_x + (next_x - grid_x) * offset, grid_y + (next_y - grid_y) * offset

def is_wall_in_range_of_distance_sensors(distances):
    all_zero = True

    for distance in distances:
        if distance != 0:
            return True
    
    return False



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

    # ball_detector = TennisBallDetector(ui=False)

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
            # =============================== Exploration
            thetaDeg = transfromTheta()
            print(f"x:{getPosition()[0]} y:{getPosition()[1]} theta:{getPosition()[2]} thetaDeg:{thetaDeg} rotDeg:{calcRotDeg()}")
            grid_x, grid_y = getGridPosition(getPosition())
            print(f"grid_x = {grid_x} grid_y = {grid_y}")

            directionsCoords = [(0,1), (-1,1), (-1,0), (-1,-1), (0,-1), (1,-1), (1,0), (1,1)]
            map2d = mapTo2D(mapbytes, MAP_SIZE_PIXELS)
            scanned_directions = scan_directions(grid_x, grid_y, map2d, 40, directionsCoords)
            legal_directions = get_legal_directions(150, 150, map2d, 4, directionsCoords)

            next_x, next_y = next_tile(grid_x, grid_y, scanned_directions, legal_directions, directionsCoords)
            dir_visualization_x, dir_visualization_y = get_direction_visualisation_coords(grid_x, grid_y, next_x, next_y, offset = 10)
            print(f"next_x = {dir_visualization_x} next_y = {dir_visualization_y}")
            show_map_in_window(mapbytes, grid_x, grid_y, dir_visualization_x, dir_visualization_y, thetaDeg)
    
            # ======================= Tennis Ball Detection

            area = 0
            ball_detected = False
            # area, ball_detected = ball_detector.detect()
            print(f"Biggest contour area: {area}    Ball detected: {ball_detected}")

            # ====================== Infra Red Distance Senseors
            
            sensors = robot.sens()
            distances = robot.sensValuesToDistance(sensors)
            print("Distances: ", distances)

            # ====================== Hierarchical Controller
            if ball_detected:
                print("Ball Found!")
                robot.stop()
            elif is_wall_in_range_of_distance_sensors(distances):
                # Avoid wall using the distance sensors
                print("Distance sensors - obstacle avoidance")
                left_wheel_velocity, right_wheel_velocity = get_wheel_velocities(distances)
                # robot.drive(left_wheel_velocity, right_wheel_velocity)
            else:
                # Explore the environment using the lidar until the tennis ball is found
                print("Environment exploration using lidar")
                # robot.driveToTarget(next_x, next_y)

        except Exception as e:
            print (e)
            traceback.print_exc()
            break



    lidar.stop()
    lidar.disconnect()
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