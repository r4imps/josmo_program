import time
import rospy
from Biti_Vabriks import *

import numpy as np

# Heading control
# Do not change the name of the function, inputs or outputs. It will break things.

def PIDController(bits , prev_e, prev_int, delta_t): #add theta_ref as input
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given).
        theta_ref (:double:) reference heading pose
        theta_hat (:double:) the current estiamted theta.
        prev_e (:double:) tracking error at previous iteration.
        prev_int (:double:) previous integral error term.
        delta_t (:double:) time interval since last call.
    returns:
        v_0 (:double:) linear velocity of the Duckiebot 
        omega (:double:) angular velocity of the Duckiebot
        e (:double:) current tracking error (automatically becomes prev_e_y at next iteration).
        e_int (:double:) current integral error (automatically becomes prev_int_y at next iteration).
    """
    index = 20 - Joonebitid.index(bits)

    # Tracking error
    e = 10 - index

    # integral of the error
    e_int = prev_int + e*delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,1.0),-1.0)

    # derivative of the error
    e_der = (e - prev_e)/delta_t

    # controller coefficients
    Kp, Ki, Kd ,v_0 = rospy.get_param("/v_pid")

    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der
    
    #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    
    return v_0, omega, e, e_int
