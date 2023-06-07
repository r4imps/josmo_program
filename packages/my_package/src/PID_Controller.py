#!/usr/bin/env python3
import rospy
from Biti_Vabriks import *

def BackOnTrack(prev_bits: list) -> float:
    index = 7.0
    for el in prev_bits:      
        if el in Paremale_90:
            index = 15.0
            print(f'paremale')
            return index
            
        elif el in Vasakule_90:
            index = 0.0
            print(f'vasakule')
            return index
            
        elif el in Joonebitid:
            index = Joonebitid.index(el)
            return index
        else:
            pass
    print(f'No reliable read in memory')
                


# Heading control
# Do not change the name of the function, inputs or outputs. It will break things.

def PIDController(prev_e, prev_int, delta_t, prev_bits: list): #add theta_ref as input
    """
    Args:
        v_0 (:double:) linear Duckiebot speed (given)
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
    prev_bits.reverse()

    if prev_bits[0] in Joonebitid:
        index = 14.0 - Joonebitid.index(prev_bits[0])
    else:
        index = BackOnTrack(prev_bits)
        rospy.sleep(0.3)
        
    # Tracking error
    e = 7.0 - index

    # integral of the error
    e_int = prev_int + e * delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2.0),-2.0)

    # derivative of the error
    e_der = (e - prev_e) / (delta_t * 100000000)  if delta_t>0 else 0.0

    # controller coefficients
    Kp , Ki, Kd, v_0 = float(rospy.get_param("/p")), float(rospy.get_param("/i")), float(rospy.get_param("/d")), float(rospy.get_param("/maxvel"))
    
    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der

    #print(f'e: {e} e_int: {e_int} e_der: {e_der} omega" {omega}')
    
    #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    
    return v_0, omega, e, e_int
