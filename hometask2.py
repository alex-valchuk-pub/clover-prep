# Information: https://clover.coex.tech/programming

import rospy
from clover import srv
from std_srvs.srv import Trigger

rospy.init_node('flight')

navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

print('Take off and hover 1 m above the ground')
navigate(x=0, y=0, z=1, frame_id='body', auto_arm=True)

# Wait for 5 seconds
rospy.sleep(5)

while True:
    try:
        print('Please, enter X and Y coordinates: X,Y')
        print('or Q for exit')
        str = input()
        if str == 'Q':
            break

        parts = str.split(',')
        if len(parts) != 2:
            print('You have made a wrong input. Please try again')
            continue

        x = int(parts[0])
        y = int(parts[1])

        print('I am going to point (X:{},Y:{})'.format(x, y))

        navigate(x=x, y=y, z=1, frame_id='aruco_map')
        rospy.sleep(x + y)
        print('Done!')
    except:
        print('Something went wrong. Please, try again...')

print('Perform landing')
land()
