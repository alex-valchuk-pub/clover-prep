# Information: https://clover.coex.tech/programming

import math
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

telem = get_telemetry()
print('Battery: {}'.format(telem.voltage))
print('Connected: {}'.format(telem.connected))


def range_callback(msg):
    global h
    h = msg.range

rospy.Subscriber('rangefinder/range', Range, range_callback)

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
