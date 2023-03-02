from my_publisher_node import Joonebitid, bits, Suund
from my_subscriber_node import speed, pub

#==========PID PARAMEETRID================    
Kp, Ki, Kd ,v_0 = rospy.get_param("/v_pid")
#==========LIST TO INDEX===================
if bits in Joonebitid:
    index = 20 - Joonebitid.index(bits)
    PID_STRT=True
#==========ROBOT PIT MODE=====================    
elif bits == '11111111':
    speed.vel_right=0
    speed.vel_left=0
    PID_STRT=False
#==========TOF DETECTION=======================  
if self.distance<0.3:
    speed.vel_right=0
    speed.vel_left=0
    PID_STRT=False
#==========RISTMIKU VALIMINE====================
if bits in Suund:
    #print('SUUNA VALIMINE -------------------------')
    flagtwo=1
    Sec_save=self.sec

if flagtwo==1 and Sec_save+10 < self.sec:  
    v_0=0.2
    if (bits in Haru) or (bits in Suund):
        speed.vel_right=0.1
        speed.vel_left=-0.1
        Sec_save=self.sec
        PID_STRT=0
    #print(f"Sec save                        :  {str(Sec_save)}")
    #print(index)
    #print("Sec "+(str(self.sec)))
    if Sec_save+30 < self.sec:
        flagtwo=0

#==============PID CONTROLLER==============================================================
PID_Time= self.sec
if PID_STRT ==True and (PID_Time>0) and PID_DELTA!=0:
    #KIIRUSE VÄHENDAMINE KUI ON LAI JOON
    if bits=='00111100':
        v_0=0.2

    error= 10 - index
    P= error
    I=prev_int+(PID_DELTA*error)
    I = max(min(I,1.0),-1.0)
    D=((error-prev_error)/PID_DELTA)
    PID= Kp*P+Ki*I+Kd*D
    #print(f'SEE ON ERROR : {error},    PREVIOUS INT : {prev_int}')
    #print(f'PID:{PID},       TIME: {PID_Time} ,  LAST_TIME: {PID_Time_Last} ')
    speed.vel_right=v_0+PID
    speed.vel_left=v_0-PID