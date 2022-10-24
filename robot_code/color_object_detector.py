import cv2 as cv
import numpy as np

# original (at ITU)   target_hsv=(34, 132, 206)
# at DR building game lab target_hsv=(26, 136, 220)


class TennisBallDetector:
    def __init__(self, target_hsv=(26, 136, 220), threshold_range=(4, 20, 100), min_contour_area=250, ui=False):
        # Detection Settings
        self.hsv_target = target_hsv
        self.hsv_range = threshold_range
        self.hsv_min, self.hsv_max = self.get_boundaries(
            self.hsv_target, self.hsv_range)
        self.min_contour_area = min_contour_area
        self.ui = ui

        # Video Capture Settings
        self.video_capture_device_index = 0
        self.video = cv.VideoCapture(self.video_capture_device_index)

        # UI
        self.key = None

        if self.ui:
            cv.namedWindow("Original", cv.WINDOW_AUTOSIZE)
            cv.namedWindow("Before", cv.WINDOW_AUTOSIZE)
            cv.namedWindow("After", cv.WINDOW_AUTOSIZE)

            cv.moveWindow("Original", 1300, 30)
            cv.moveWindow("Before", 50, 30)
            cv.moveWindow("After", 700, 30)

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
        contours, hierarchy = cv.findContours(
            morph_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
        bgr_img = cv.cvtColor(morph_img, cv.COLOR_GRAY2BGR)

        biggest_contour_area = 0

        if len(contours) > 0:
            biggest_contour = max(contours, key=cv.contourArea)
            biggest_contour_area = cv.contourArea(biggest_contour)
            line_color = (0, 0, 255)
            cv.drawContours(bgr_img, biggest_contour, -1,
                            line_color, 3)  # -1 = all contours

        if self.ui:
            cv.imshow("After", bgr_img)
            self.key = cv.waitKey(1)

        return biggest_contour_area, biggest_contour_area > self.min_contour_area


if __name__ == "__main__":
    # run()
    detector = TennisBallDetector(ui=True)

    while detector.key != ord("q"):
        area, detected = detector.detect()
        print(f"Biggest contour area: {area}    Ball detected: {detected}")
