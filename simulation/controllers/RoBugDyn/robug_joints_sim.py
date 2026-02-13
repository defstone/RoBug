from time import sleep
from robug_constants_dyn import constants as c
        
class rbjoints:
    
    def __init__(self, iLegID):
        self.ID   = iLegID
        self.name = c._GAIT_NAME[iLegID]
        ID0 = (2*iLegID)
        ID1 = (2*iLegID)+1        
        self.init_joints()
    
    def init_joints(self):
        pass

    def deinit(self):
        pass         
                   
    def safe_limits(self, iTicks):
        pass
            
    def set_angle_sid(self, sid, iAngleInTicks):        
        pass

    def set_angles(self, iDeltaTicks, iGammaTicks):
        pass       
