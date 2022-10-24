import pickle

# SLAM constants
MAP_SIZE_PIXELS = 250
MAP_SIZE_METERS = 15
LIDAR_DEVICE = '/dev/ttyUSB0'

MAP_SIZE_MM = MAP_SIZE_METERS * 1000
TILE_SIZE_MM = MAP_SIZE_MM / MAP_SIZE_PIXELS

directionsCoords = [(0,1), (-1,1), (-1,0), (-1,-1), (0,-1), (1,-1), (1,0), (1,1)]


def printMap(map):
    for y in range(MAP_SIZE_PIXELS):
        for x in range(MAP_SIZE_PIXELS):
            wall_probability = map[x + y * MAP_SIZE_PIXELS]
            print(wall_probability)

            map_char = ' '

            if wall_probability < 127:
                map_char = '.'
            elif wall_probability > 127:
                map_char = '*'

            # print(f"{map_char}, ", end="")

        print()

# Given a flatten (1D list) of the map, 
# returns a 2D version of it where:
# map[Y][X] == map[row][column]
def mapTo2D(map, division=MAP_SIZE_PIXELS):
    return np.array_split(np.array(map), division)

# Scan in the 8 directions surrounding the robot.
# Returns the sum of the probabilities values within
# the specified radius of the robot.
def scan_directions(x, y, myMap, radius):
    counters = dict.fromkeys(directionsCoords, 0)
    for dx, dy in counters:
        try:
            for i in range(1, radius+1):
                if y+dy*i < 0 or x+dx*i < 0:
                    counters[(dx,dy)] += 255
                else:
                    counters[(dx,dy)] += myMap[y+dy*i][x+dx*i]
        except IndexError:
            pass
    return counters.values()




def isWall(value):
    return True if value > 127 else False

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
def get_legal_directions(x, y, map, min_wall_distance):
    newMap= list.copy(map)
    newMap.reverse()
    directions = [1,1,1,1,1,1,1,1]  # [T, TL, L, BL, B, BR, R, TR]
    for idx, directionCoords in enumerate(directionCoords):
        if isWallPresent(x,y, directionCoords, min_wall_distance, newMap):
            directions[idx] = 0
    return directions

# Returns the new position to be used for the robot based on
# which direction has the least amount of wall "presence".
def next_tile(x,y, scanned_directions, legal_directions):
    idx = scanned_directions.index(min(scanned_directions))
    if legal_directions[idx] == 1:
        direction = directionsCoords[idx]
    else:
        dirsCopy = list.copy(scanned_directions)
        dirsCopy[idx] = 1000000
        direction = next_tile(x, y, dirsCopy, legal_directions)
    return (x + direction[0], y + direction[1])


if __name__ == "__main__":
    infile = open("mapbytes", 'rb')
    mapbytes = pickle.load(infile)
    infile.close()

    get_legal_directions(150, 150, mapbytes, 2)
    # print(type(mapbytes))
    # printMap(mapbytes)

    
    myMap = [1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,
            1,1,1,1,1,1,1,
            1,1,1,1,1,1,1]
    # printMap(mapbytes)
    x = mapTo2D(myMap, 7)