"""#==========RISTMIKU VALIMINE====================
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
        flagtwo=0"""