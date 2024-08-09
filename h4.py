# Information: https://clover.coex.tech/programming

import math
import rospy
import cv2
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from colorama import Fore

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
    
# read about:
#   - hsv
#   - cv masks (red, yellow, brown, etc.)
#   - cv moments: https://docs.opencv.org/4.x/dd/d49/tutorial_py_contour_features.html
#   - fire_debug: http://127.0.0.1/clover/topics.html
#   - home/clover/examples/red_circle.py
#   - http://127.0.0.1/clover/docs/ru/innopolis_open_L22_AERO.html
#   - S

# EXPECTED
# WATER     APPLES
# CHERRIES  POTATOES

# ACTUAL
# CHERRIES  APPLES
# WATER     POTATOES
goods_to_zones = {
    "WATER": "00",
    "APPLES": "01",
    "CHERRIES": "10",
    "POTATOES": "11"
}
zones_to_goods = {
    "00": "WATER",
    "01": "APPLES",
    "10": "CHERRIES",
    "11": "POTATOES"
}

telem = get_telemetry()
print('Battery: {}'.format(telem.voltage))
print('Connected: {}'.format(telem.connected))


def range_callback(msg):
    global h
    h = msg.range

rospy.Subscriber('rangefinder/range', Range, range_callback)

def get_code_zone(xc, yc):
    zone_x = -1
    zone_y = -1
    
    if xc <= 89:
        zone_x = 0
    elif xc <= 200:
        zone_x = 1

    if yc <= 100:
        zone_y = 0
    elif yc <= 200:
        zone_y = 1

    zone_code = "{}{}".format(zone_x, zone_y)
          
    if zone_code in zones_to_goods:
        return zones_to_goods[zone_code]
    
    return None

@long_callback
def image_callback(msg):
    img = bridge.imgmsg_to_cv2(msg, 'bgr8')
    barcodes = pyzbar.decode(img, [ZBarSymbol.QRCODE])
    
    image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))

    for barcode in barcodes:
        b_data = barcode.data.decode('utf-8')
        #b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2

        # we are not interested in other barcodes atm.
        if b_data not in goods_to_zones:
            print(Fore.WHITE + 'Found not interesting {} QRcode'.format(b_data))
            continue

        current_zone = get_code_zone(xc, yc)
        actual_zone = b_data
        
        if actual_zone == current_zone:
            msg_color = Fore.GREEN
            rect_color = (0, 255, 0)
            object_state = "correct"
        else:
            msg_color = Fore.RED
            rect_color = (0, 0, 255)
            object_state = "incorrect"

        cv2.rectangle(img, (x, y), (x + w, y + h), rect_color, 3)
        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        print(msg_color + 'Found {} located at x={}, y={} in a zone for {}: {}'.format(b_data, xc, yc, actual_zone, object_state) + Fore.WHITE)

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
