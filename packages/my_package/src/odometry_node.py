#!/usr/bin/env python3

from sensor_msgs.msg  import Range
from duckietown.dtros import DTROS, NodeType
from duckietown_msgs.msg import WheelEncoderStamped
import rospy
import numpy as np

class ODOMETRY(DTROS):
    def __init__(self, node_name):
        super(ODOMETRY, self).__init__(node_name=node_name, NodeType =NodeType.GENERIC)
        
        self.pub = rospy.Publisher('josmo/odometry_node/data')

        self.Delta_x=0
        self.Delta_y=0
        self.flag = 1

    def Callback_R_Encoder(self,data):
        self.R_encoder = data.data

    def Callback_L_Encoder(self,data):
        self.L_encoder = data.data

    def run(self):
        N_tot= 135
        R=0.03424
        baseline_wheel2wheel= 0.1        

                #================Encoder Zeroing===================
        if (self.L_encoder>0 or self.R_encoder>0) and self.flag == 0:
            Display_L_en=self.L_encoder-Save_L_en
            Display_R_en=self.R_encoder-Save_R_en
        elif self.flag == 1 and (self.L_encoder != 0 or self.R_encoder!=0):
            Save_L_en = self.L_encoder
            Save_R_en = self.R_encoder
            self.flag = 0

        L_Rotation= Display_L_en * ((2*np.pi)/N_tot)
        #print(f"The left wheel rotated: {L_Rotation} degrees")

        R_Rotation= Display_R_en * ((2*np.pi)/N_tot)
        #print(f"The right wheel rotated: {R_Rotation} degrees")

        L_Distance= R*L_Rotation
        #print(f"The left wheel travel: {round(L_Distance,4)} m")

        R_Distance= R*R_Rotation
        #print(f"The righ wheel travel: {round(R_Distance,4)} m")

        Delta_A=(R_Distance+L_Distance)/2
        #print(f"Delta distance:===== {round(Delta_A,3)} =====")

        Delta_null= (R_Distance-L_Distance)/baseline_wheel2wheel
        #print(f"Delta rotation: {round(Delta_null,3)}")

        self.Delta_x=Delta_A*np.cos(Delta_null)
        #print(f"Delta X:================= {round(Delta_x,2)} ================")

        self.Delta_y=Delta_A*np.sin(Delta_null)
        #print(f"Delta Y:================= {round(Delta_y,2)} ================")