# Information: https://clover.coex.tech/programming

import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='aruco_map', auto_arm=True)

# Wait for 5 seconds
rospy.sleep(5)

while 1:
    try:
        print('Enter X:')
        x = int(input())

        print('Enter Y:')
        y = int(input())

        print('The following coordinates are entered: X={}, Y={}'.format(x, y))

        navigate(x=x, y=y, z=1, frame_id='aruco_map')

        rospy.sleep(5)
    except:
        print('You have entered wrong numbers :( ')
        rospy.sleep(1)
        break

print('Perform landing')
land()
