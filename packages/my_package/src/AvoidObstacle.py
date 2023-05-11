import time
import rospy

def AvoidObstacle(start_time):
    v_0 = float(rospy.get_param("/maxvel"))
    avoiding_obstruction = True

    if time.time() - start_time <= 2.5:
        vel_right = v_0
        vel_left = v_0 + (time.time() - start_time) ** 2

    elif time.time() - start_time >= 5:
        vel_right = v_0
        vel_left = v_0
    
    else:
        vel_right = v_0 + ((start_time + 5) - time.time()) ** 2
        vel_left = v_0


    return vel_right, vel_left, avoiding_obstruction
