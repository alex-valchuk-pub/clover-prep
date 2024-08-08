# Information: https://clover.coex.tech/programming

import math
import rospy
import cv2
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

from pyzbar import pyzbar
from pyzbar.pyzbar import ZBarSymbol
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from clover import long_callback

rospy.init_node('flight')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
image_pub = rospy.Publisher('~fire_debug', Image, queue_size=1)


telem = get_telemetry()
print('Battery: {}'.format(telem.voltage))
print('Connected: {}'.format(telem.connected))


def range_callback(msg):
    global h
    h = msg.range

rospy.Subscriber('rangefinder/range', Range, range_callback)

@long_callback
def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV) [20:200,20:300]

    red_low1 = (0, 110, 150)
    red_high1 = (7, 255, 255)
    
    yellow_orange_low = (38, 110, 150)
    yellow_orange_high= (52, 110, 150)

    #гречиха
    brown_low = (23, 50, 50)
    brown_high= (37, 50, 50)

    red_mask = cv2.inRange(img_hsv, red_low1, red_high1)
    yellow_orange_mask = cv2.inRange(img_hsv, yellow_orange_low, yellow_orange_high)
    brown_mask = cv2.inRange(img_hsv, brown_low, brown_high)

    red_contours, red_hierarchy = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

    #if red_mask[119][159] == 255:
    #    shape = shape_recog(red_mask)

    #elif yellow_orange_mask[119][159] == 255:
    #    shape = shape_recog(yellow_orange_mask)

    #elif brown_mask[119][159] == 255:
    #    shape = shape_recog(brown_mask)

    #else:
    #    shape = 'undefined'
    #    color = 'undefined'

    #if shape == 'red':
    #    print ('red circle detected')
    #if shape == 'brown':
    #    culture = "greshiha"
    #if shape == 'yellow_orange':
    #    culture = "pshenitsa"

    
# read about:
#   - hsv
#   - cv masks (red, yellow, brown, etc.)
#   - cv moments: https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
#   - fire_debug: http://127.0.0.1/clover/topics.html
#   - home/clover/examples/red_circle.py
#   - http://127.0.0.1/clover/docs/ru/innopolis_open_L22_AERO.html
#   - S

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)


print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 5 seconds
rospy.sleep(5)

while True:
    try:
        print('Please, enter X,Y,Z coordinates.')
        print('or R for current range')
        print('or Q for exit')
        str = input()
        if str == 'R':
            print('Rangefinder distance: {}'.format(h))
            continue

        if str == 'Q':
            break

        parts = str.split(',')
        if len(parts) != 3:
            print('You have made a wrong input. Please try again')
            continue

        x = int(parts[0])
        y = int(parts[1])
        z = int(parts[2])

        print('I am going to point (X:{},Y:{},Z:{})'.format(x, y, z))

        navigate(x=x, y=y, z=z, frame_id='aruco_map')
        rospy.sleep(5)
        print('Done!')

    except:
        print('Something went wrong. Please, try again...')

print('Perform landing')
land()
