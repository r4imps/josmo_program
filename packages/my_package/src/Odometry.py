"""

flag=1
Save_L_en=0
Save_R_en=0


#================Encoder Zeroing===================
if (self.L_encoder>0 or self.R_encoder>0) and flag == 0:
    Display_L_en=self.L_encoder-Save_L_en
    Display_R_en=self.R_encoder-Save_R_en
elif flag == 1 and (self.L_encoder != 0 or self.R_encoder!=0):
    Save_L_en = self.L_encoder
    Save_R_en = self.R_encoder
    flag = 0

#==================ODOMEETRIA ARVUTUSKÄIK====================
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

Delta_null= (R_Distance-L_Distance)/Back2L
#print(f"Delta rotation: {round(Delta_null,3)}")

Delta_x=Delta_A*np.cos(Delta_null)
#print(f"Delta X:================= {round(Delta_x,2)} ================")

Delta_y=Delta_A*np.sin(Delta_null)
#print(f"Delta Y:================= {round(Delta_y,2)} ================")"""