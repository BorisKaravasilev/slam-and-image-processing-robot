from collections import deque
from unicodedata import name #like list, but faster append and pop
from imutils.video import VideoStream
import numpy as np
import argparse
import cv2
import imutils
import time

def getBallCoords(laptopWebcam=False):
    # define the lower and upper boundaries of the "green"
    # ball in the HSV color space, then initialize the
    # list of tracked points
    colourTresholdLower = (0, 157, 167)
    colourTresholdUpper = (134, 255, 255)
    tailLength = 64
    pts = deque(maxlen=tailLength)
    # grab the reference to the webcam
    vs = VideoStream(src=0).start()
    # allow the camera or video file to warm up
    time.sleep(5.0)


    # keep looping
    while True:
        # grab the current frame
        frame = vs.read()
        # resize the frame, blur it, and convert it to the HSV
        # color space
        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # construct a mask for the color "green", then perform
        # a series of dilations and erosions to remove any small
        # blobs left in the mask
        mask = cv2.inRange(hsv, colourTresholdLower, colourTresholdUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = imutils.grab_contours(contours)
        center = None
        # only proceed if at least one contour was found
        if len(contours) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            print("Coordinates of ball: ", center)
            # only proceed if the radius meets a minimum size
            if radius > 10:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                cv2.circle(frame, (int(x), int(y)), int(radius),
                    (0, 255, 255), 2)
                cv2.circle(frame, center, 5, (0, 0, 255), -1)
        else:
            print("NO BALL FOUND!!!")
                
        # update the points queue
        pts.appendleft(center)
        
        if laptopWebcam:
            # loop over the set of tracked points
            for i in range(1, len(pts)):
                # if either of the tracked points are None, ignore
                # them
                if pts[i - 1] is None or pts[i] is None:
                    continue
                # otherwise, compute the thickness of the line and
                # draw the connecting lines
                thickness = int(np.sqrt(tailLength / float(i + 1)) * 2.5)
                cv2.line(frame, pts[i - 1], pts[i], (0, 0, 255), thickness)
            # show the frame to our screen
            cv2.imshow("Frame", frame)
            # if the 'q' key is pressed, stop the loop
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
        else:
            return center, radius
            
    # stop the camera video stream
    vs.stop()
    # close all windows
    cv2.destroyAllWindows()






if __name__ == "__main__":
    getBallCoords(laptopWebcam=True)