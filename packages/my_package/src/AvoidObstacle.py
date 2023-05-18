import time
import rospy
rospy.set_param("/maxvel", 0.3)
def AvoidObstacle(start_time):
    v_0 = float(rospy.get_param("/maxvel"))
    avoiding_obstruction = True

    if time.time() - start_time <= 2.5:
        vel_right = v_0
        vel_left = v_0 + ((time.time() - start_time) ** 2) / 20

    elif 2.5 < time.time() - start_time <= 4:
        vel_right = v_0
        vel_left = v_0
    
    else:
        vel_right = v_0 + v_0 * 0.2
        vel_left = v_0
        avoiding_obstruction = False
        
    return vel_right, vel_left, avoiding_obstruction
