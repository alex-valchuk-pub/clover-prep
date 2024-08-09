# Information: https://clover.coex.tech/programming

import math
import rospy
import cv2
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from colorama import Fore

from pyzbar import pyzbar
from pyzbar.pyzbar import ZBarSymbol
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from clover import srv
from clover import long_callback
from clover.srv import SetLEDEffect

rospy.init_node('flight')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)
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

goods_heights = {
    "WATER": 0,
    "APPLES": 0,
    "CHERRIES": 0,
    "POTATOES": 0
}

telem = get_telemetry()
print('Battery: {}'.format(telem.voltage))
print('Connected: {}'.format(telem.connected))

def range_callback(msg):
    global h
    h = msg.range

rospy.Subscriber('rangefinder/range', Range, range_callback, queue_size=1)

def get_code_zone(xc, yc):
    zone_x = -1
    zone_y = -1
    
    if xc <= 2.5:
        zone_x = 0
    elif xc <= 5:
        zone_x = 1

    if yc <= 2.5:
        zone_y = 0
    elif yc <= 5.5:
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

        telem = get_telemetry('aruco_map')
        current_zone = get_code_zone(telem.x, telem.y)
        actual_zone = b_data
        
        if actual_zone == current_zone:
            msg_color = Fore.GREEN
            rect_color = (0, 255, 0)
            object_state = "correct"
            set_effect(r=255, g=255, b=0)  # fill strip with red color
        else:
            msg_color = Fore.RED
            rect_color = (0, 0, 255)
            object_state = "incorrect"
            set_effect(r=255, g=0, b=0)  # fill strip with red color
        
        global current_good
        current_good = b_data

        cv2.rectangle(img, (x, y), (x + w, y + h), rect_color, 3)
        image_pub.publish(bridge.cv2_to_imgmsg(img, 'bgr8'))
        print(msg_color + 'Found {} in a zone for {}: {}'.format(b_data, current_zone, object_state) + Fore.WHITE)

    rospy.sleep(1)

image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)

def record_height():
    if current_good not in goods_heights:
        return
    
    range = rospy.wait_for_message('rangefinder/range', Range).range
    telem = get_telemetry(frame_id='map')
    goods_heights[current_good] = telem.z - range
    print('{}{}'.format(current_good, goods_heights[current_good]))
    print('Z = {}'.format(telem.z))
    print('H = {}'.format(h))

def record_report():
    my_file = open("D_report_fly.txt", "w+")
    highest_good = ""
    highest = 0
    i = 0
    for key in goods_heights:
        height = goods_heights[key]
        my_file.write('object {}: {}, height = {}\n'.format(i, key, height))

        i += 1
        if height > highest:
            highest = height
            highest_good = key

    my_file.write('The heighest object is {}'.format(highest_good))
    my_file.close()

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 5 seconds
rospy.sleep(5)

while True:
    try:
        print('Please, enter X,Y,Z coordinates.')
        print('or H for current height')
        print('or R for current report')
        print('or Q for exit')
        str = input()
        if str == 'H':
            record_height()
            continue

        if str == 'R':
            record_report()
            continue

        if str == 'Q':
            break

        parts = str.split(',')
        if len(parts) != 3:
            print('You have made a wrong input. Please try again')
            continue

        x = float(parts[0])
        y = float(parts[1])
        z = float(parts[2])

        print('I am going to point (X:{},Y:{},Z:{})'.format(x, y, z))

        navigate(x=x, y=y, z=z, frame_id='aruco_map')
        rospy.sleep(5)
        print('Done!')

    except:
        print('Something went wrong. Please, try again...')

print('Perform landing')
land()
