from time import sleep
from picamera import PiCamera

camera = PiCamera()

def testCamera():
    print('Camera_test')
    camera.start_preview()
    sleep(5)
    camera.resolution = (320, 240)
    camera.framerate = 24
    sleep(2)
    image = np.empty((240 * 320 * 3,), dtype=np.uint8)
    camera.capture(image, 'bgr')
    cv2.imwrite('out.png', image)
    camera.stop_preview()
    print ('Saved image to out.png')