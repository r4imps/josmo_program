
import time
import rospy


I=0
error=0
prev_int=0
PID_Time_Last=0


def PID_STRT(JooneInput):
    
    Joonebitid=['10000000',
    '11000000',
    '01000000',
    '01100000',
    '00100000',
    '00110000',
    '00010000',
    '00011000',
    '00001000',
    '00001100',
    '00000100',
    '00000110',
    '00000010',
    '00000011',
    '00000001']
    

 
#==========PID PARAMEETRID================
#   
    global I
    global prev_error
    global error
    global prev_int
    global PID_DELTA
    global PID_Time_Last
    

    rospy.set_param("/v_pid", [0.045 ,0.022 ,0.25 ,0.4]) 
    Kp,Ki,Kd,v_0 = rospy.get_param("/v_pid")
    PID_STRT=False

        #==========LIST TO INDEX===================
    if JooneInput in Joonebitid:
        index = 14 - Joonebitid.index(JooneInput)
        print(f'index on: {index}')
        PID_STRT=True
    #==========ROBOT PIT MODE=====================    
    elif JooneInput == '11111111':
        vel_to_right=0
        vel_to_left=0
        PID_STRT=False
        return vel_to_right , vel_to_left
        


    
    #==============PID CONTROLLER==============================================================
    PID_Time= time.time()
    
  

        


    if PID_STRT ==True and PID_Time!=0 and PID_DELTA!=0:
        print("PID ACTIVE")
        #KIIRUSE VÄHENDAMINE KUI ON LAI JOON
        if JooneInput=='00111100':
            v_0=0.2

        error= 7 - index
        P= error
        I=prev_int+(PID_DELTA*error)
        I = max(min(I,1.0),-1.0)
        D=((error-prev_error)/PID_DELTA)
        PID= Kp*P+Ki*I+Kd*D
        #print(f'SEE ON ERROR : {error},    PREVIOUS INT : {prev_int}')
        #print(f'PID_START{PID_STRT}       TIME: {PID_Time} ,  LAST_TIME: {PID_Time_Last} ')
        vel_to_right=v_0+PID
        vel_to_left=v_0-PID
        
        return vel_to_right , vel_to_left
    PID_DELTA= PID_Time-PID_Time_Last
    PID_Time_Last=PID_Time
    prev_int=I
    prev_error=error
    
    

#!/usr/bin/env python3
import rospy
from Biti_Vabriks import *

# Heading control
# Do not change the name of the function, inputs or outputs. It will break things.
def BackOnTrack(prev_bits) -> float:
    index = 7
    for last_reliable_read in prev_bits[::-1]:
        if last_reliable_read != '00000000':
            
            if last_reliable_read in Paremale_90:
                index = 15.0
                print(f'paremale')
                return index
            elif last_reliable_read in Vasakule_90:
                index = 0.0
                print(f'vasakule')
                return index
            else:
                #print(f'No reliable read in memory')
                pass
    return index



def PIDController(bits, prev_e, prev_int, delta_t, prev_bits): #add theta_ref as input
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
    if bits in Joonebitid:
        index = 14 - Joonebitid.index(bits)
        #print(f'Read data: {bits}')
    else:
        if bits in (Paremale_90 + Vasakule_90):
            index = BackOnTrack(bits)
        else:
            index = BackOnTrack(prev_bits)
        #print(f'INDEX NOT FOUND     BITS: {bits}')
    # Tracking error
    e = 7 - index

    # integral of the error
    e_int = prev_int + e * delta_t

    # anti-windup - preventing the integral error from growing too much
    e_int = max(min(e_int,2.0),-2.0)

    # derivative of the error
    e_der = (e - prev_e) / (delta_t * 10000000) if delta_t>0 else 0.0

    # controller coefficients
    Kp , Ki, Kd, v_0 = float(rospy.get_param("/p")), float(rospy.get_param("/i")), float(rospy.get_param("/d")), float(rospy.get_param("/maxvel"))
    
    # PID controller for omega
    omega = Kp*e + Ki*e_int + Kd*e_der

    #print(f'e: {e} e_int: {e_int} e_der: {e_der} omega" {omega}')
    
    #print(f"\n\nDelta time : {delta_t} \nE : {np.rad2deg(e)} \nE int : {e_int} \nPrev e : {prev_e} \nU : {u} \nTheta hat: {np.rad2deg(theta_hat)} \n")
    
    return v_0, omega, e, e_int
