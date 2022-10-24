#!/usr/bin/python3
from cmath import pi
import os

# initialize asebamedulla in background and wait 0.3s to let
# asebamedulla startup
os.system("(asebamedulla ser:name=Thymio-II &) && sleep 0.3")
import matplotlib.pyplot as plt
from time import sleep
import dbus
import dbus.mainloop.glib
from threading import Thread

from ballTracking import getBallCoords


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


def chaseBall():
    coordinates, radius = getBallCoords(show=False)
    x_ball = coordinates[0]
    correction = (300 - x_ball)/10  # 300 is middle of coordinates axis
    left_wheel_velocity = base_velocity + correction
    right_wheel_velocity = base_velocity - correction
    return left_wheel_velocity, right_wheel_velocity


    return

# ------------------ Main -------------------------

def main():
    robot = Thymio()

    # count = 1
    # left_wheel_velocity = 200
    # right_wheel_velocity = 200
    # # Get initial sensor values


    # while count < 500:

    #     robot.drive(left_wheel_velocity,right_wheel_velocity)
    #     robot.sens()
    #     count += 1

    # sensors = robot.sens()
    # while not robot.inRangeOfWall(sensors):
    #     robot.drive2cm(left_wheel_velocity, right_wheel_velocity)
    #     sensors = robot.sens()

    for i in range(10000):
        sensors = robot.sens()
        # print("Sensor values:\n", sensors)

        distances = robot.sensValuesToDistance(sensors)
        print("Distances: ", distances)

        #left_wheel_velocity, right_wheel_velocity = get_wheel_velocities(distances)
        left_wheel_velocity, right_wheel_velocity = chaseBall()
        robot.drive(left_wheel_velocity, right_wheel_velocity)


    robot.stop()


# ------------------- Main ------------------------

# ------------------- loop ------------------------

def loop():
    # robot.drive(200, 200)
    print("nothing to do.. zzz")
    sleep(5)
    # robot.stop()


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
        print("asebamodulla killed")
