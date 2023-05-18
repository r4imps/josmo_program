from smbus2 import SMBus
import rospy
rospy.set_param("/v_pid", [0.068 ,0.012 ,0.22 ])

def get_line_values():
    bus = SMBus(1)
    read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)

    line_values = []
    for i, value in enumerate(read):
        if value =='1':
            line_values.append(i + 1)
    
    return line_values

def get_theta():
    bus = SMBus(1)
    read = bin(bus.read_byte_data(62, 17))[2:].zfill(8)

    line_values = []
    
    for i, value in enumerate(read):
        if value =='1':
            line_values.append(i + 1)
    if len(line_values) >= 1:
        theta_hat = sum(line_values)/len(line_values)
    if len(line_values) == 0:
        theta_hat = 4

    return theta_hat

def pid_controller(t0,t1):
        delta_t = 1
        pose_estimation = 4.5
        prev_int = 0
        
        e = pose_estimation - get_theta()
        e_int = prev_int + e*delta_t
        prev_int = e_int                                       
        prev_e = e                                             
        e_int = max(min(e_int,2),-2)                            
        e_der = (e - prev_e)/delta_t  
                                  
        
        Kp,Ki,Kd = rospy.get_param("/v_pid")
        
        """Kp=0.045
        Ki=0.022
        Kd=0.25"""
        

        delta_t = t0-t1
        omega = Kp*e + Ki*e_int + Kd*e_der 
                     
        return omega