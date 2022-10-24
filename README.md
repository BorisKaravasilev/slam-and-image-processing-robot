# Search and Rescue Robot (Lidar + Image Processing)

_ITU - Autumn 2022 - Advanced Robotics - Group 3_

**Team members:**

- Balázs Tóth (balazs.toth818@gmail.com)
- Boris Karavasilev (karavasilev.boris@gmail.com)
- Michele Imbriani (imbr.mic@gmailc.com)

The full report containing the results of our experiments can be found [here](./report.pdf).

## Introduction

The goal of this mini-project was to become familiar with different types of sensors, learn about the implementation of robotics simulations and gain experience with SLAM (Simultaneous Localization And Mapping) on a complex system consisting of multiple components. We used a Thymio II robot which was equipped with several infrared distance sensors, a lidar, a camera and a Raspberry Pi as a controller.

To gain practical experience with the topics taught in the lectures we had to program our robot to complete a search and rescue mission. Our robot had to explore a classroom environment and find a tennis ball using its camera. After the tennis ball was found the robot had to return to its starting position. During our tests, we found out that our robot is able to locate the tennis ball but due to the inconsistent map of the room it was not feasible to return to the starting position.

## Lidar Scanning and Exploration

On the image below you can see the robot represented by a red ellipse and the direction that shall be explored next indicated by a blue dot. White areas of the map symbolize free space, black symbolizes walls and gray areas are not yet explored.

![map](<./experiments/experiment1%20(static%20environment%20drift)/before.png>)

Below is the real environment that was scanned in this particular case.

![environment](<./experiments/experiment1%20(static%20environment%20drift)/environment.jpg>)

## Tennis Ball Detection

Below is a series of screenshots showing the video feed on the right and the processed image on the left. The detection was tested at distances starting from 30 cm and increasing by 30 cm up to the furthest point of 210 cm where the detection fails.

### Processing Steps

1. **Gaussian Blur** - to make the ball more homogenous
2. **RGB to HSV conversion** - to isolate the hue of the color and reduce the influence of lighting
3. **Color Segmentation** - to filter out non-green colors
4. **Morphological Operations** - to remove noise and join the remaining white areas
5. **Contour Finding** - to find the contour with the biggest area

The source code can be found in the [color_object_detector.py](./robot_code/color_object_detector.py) file and run on a PC with a web camera, installed Python 3.x and the package opencv-python through.

```
> pip install opencv-python
```

![competition map solution](<./experiments/experiment6%20(ball%20detection%20good%20light)/detection-good-light.png>)
