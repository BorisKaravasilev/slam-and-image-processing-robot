import shapely
from shapely.geometry import LinearRing, LineString, Point
from numpy import sin, cos, pi, sqrt
from random import random

# A prototype simulation of a differential-drive robot with one sensor

# Constants
###########
R = 0.022  # radius of wheels in meters
L = 0.094  # distance between wheels in meters

W = 2.0  # width of arena
H = 2.0  # height of arena

robot_timestep = 0.1  # 1/robot_timestep equals update frequency of robot (100 ms)
simulation_timestep = 0.01  # timestep in kinematics sim (probably don't touch..)

# the world is a rectangular arena with width W and height H
world = LinearRing([(W / 2, H / 2), (-W / 2, H / 2), (-W / 2, -H / 2), (W / 2, -H / 2)])

# Variables 
###########

x = 0.0  # robot position in meters - x direction - positive to the right
y = 0.0  # robot position in meters - y direction - positive up
q = 0.0  # robot heading with respect to x-axis in radians

base_velocity = pi / 20

left_wheel_velocity = base_velocity  # robot left wheel velocity in radians/s
right_wheel_velocity = base_velocity  # robot right wheel velocity in radians/s


# Kinematic model
#################
# updates robot position and heading based on velocity of wheels and the elapsed time
# the equations are a forward kinematic model of a two-wheeled robot - don't worry just use it
def simulationstep():
    global x, y, q

    for step in range(int(robot_timestep / simulation_timestep)):  # step model time/timestep times
        v_x = cos(q) * (R * left_wheel_velocity / 2 + R * right_wheel_velocity / 2)
        v_y = sin(q) * (R * left_wheel_velocity / 2 + R * right_wheel_velocity / 2)
        omega = (R * right_wheel_velocity - R * left_wheel_velocity) / (2 * L)

        x += v_x * simulation_timestep
        y += v_y * simulation_timestep
        q += omega * simulation_timestep


# Our functions
#################
def read_sensors():
    return 0, 0, 0, 0.05, 0.05


def get_wheel_velocities():
    """
    Simple obstacle avoiding controller that is biased to steer left.
    """

    l2, l1, m, r1, r2 = sensor()

    correction = l2 + 2*l1 + 3.5*m - 2*r1 - r2
    correction = correction * 25
    left_wheel_velocity = base_velocity + correction
    right_wheel_velocity = base_velocity - correction
    return left_wheel_velocity, right_wheel_velocity

def sensor():
    initial_deg = [-pi/4, -pi/8, 0, pi/8, pi/4]
    distances = []
    for deg in initial_deg:
        ray = LineString([(x, y), (x+cos(q+deg)*2*(W+H),(y+sin(q+deg)*2*(W+H))) ])  # a line from sensor to a point outside arena in direction of q
        s = world.intersection(ray)
        distance = sqrt((s.x-x)**2+(s.y-y)**2)                    # distance to wall for sensor 0
        if distance > 0.30:
          distance = 0

        distances.append(distance)

    # print(distances)
    return tuple(distances)



# Simulation loop
#################
file = open("trajectory.dat", "w")

# 5000 * 100 = 500 000 ms = 500 s

for cnt in range(50000):
    # simple single-ray sensor
    ray = LineString([(x, y), (x + cos(q) * 2 * (W + H), (
            y + sin(q) * 2 * (W + H)))])  # a line from robot to a point outside arena in direction of q
    s = world.intersection(ray)
    distance = sqrt((s.x - x) ** 2 + (s.y - y) ** 2)  # distance to wall

    # Our controller
    left_wheel_velocity, right_wheel_velocity = get_wheel_velocities()
    print(left_wheel_velocity, right_wheel_velocity)

    # step simulation
    simulationstep()

    # check collision with arena walls
    if (world.distance(Point(x, y)) < L / 2):
        break

    if cnt % 50 == 0:
        file.write(str(x) + ", " + str(y) + ", " + str(cos(q) * 0.2) + ", " + str(sin(q) * 0.2) + "\n")

file.close()
