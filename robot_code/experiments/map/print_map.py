import pickle
import cv2 as cv
import numpy as np

# SLAM constants
MAP_SIZE_PIXELS = 250
MAP_SIZE_METERS = 15
LIDAR_DEVICE = '/dev/ttyUSB0'

MAP_SIZE_MM = MAP_SIZE_METERS * 1000
TILE_SIZE_MM = MAP_SIZE_MM / MAP_SIZE_PIXELS


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

def save_map_to_file(mapbytes, filename):
    map_img = np.array(mapbytes).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    cv.imwrite("map.png", map_img)


def show_map_in_window(mapbytes, robot_x, robot_y, angle):
    map_img = np.array(mapbytes).reshape(MAP_SIZE_PIXELS, MAP_SIZE_PIXELS)
    bgr_map_img = cv.cvtColor(map_img, cv.COLOR_GRAY2BGR)
    bgr_map_img = cv.ellipse(bgr_map_img, (robot_x, robot_y), (8, 4), angle, 0, 360, (0, 0, 255), 3)
    cv.namedWindow("Map", cv.WINDOW_AUTOSIZE)
    cv.imshow("Map", bgr_map_img)
    cv.waitKey(0)


if __name__ == "__main__":
    infile = open("mapbytes", 'rb')
    mapbytes = pickle.load(infile)
    infile.close()

    # Console
    # printMap(mapbytes)

    # To file
    # save_map_to_file(mapbytes, "map.png")
    # To Window
    show_map_in_window(mapbytes, 150, 150, 45)