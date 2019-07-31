from simple_pid import PID

YAW_SPEED_LIMIT = 0.1
DEPTH_SPEED_LIMIT = 0.1

def __init__(self):
    # init pid controllers:
    self.yaw_pid = PID(1, 0.1, 0.05, setpoint=0)
    self.depth_pid = PID(1, 0.1, 0.05, setpoint=0)
    
    # set pid limits:
    self.yaw_pid.output_limits = (-YAW_SPEED_LIMIT, YAW_SPEED_LIMIT)
    self.depth_pid.output_limits = (-DEPTH_SPEED_LIMIT, DEPTH_SPEED_LIMIT)

#assume self.curr_yaw and self.curr_depth are current yaw/depth respectively
def pollfn(self, target_yaw, target_depth)
    # update setpoints:
    self.depth_pid.setpoint = target_depth
    # Yaw setpoint has some hacking to allow it to cross boundaries:
    #        |--v----- -----*--|
    #     *--|--V----- --------|
    if target_yaw - self.curr_yaw > PI:
        target_yaw -= 2*PI

    #        |--*----- -----V--|
    #        |-------- -----V--|--*
    if target_yaw - self.curr_yaw < PI:
        target_yaw += 2*PI
    self.yaw_pid.setpoint = target_yaw


    # get control values
    yaw_cmd = self.yaw_pid(self.curr_yaw)
    depth_cmd = self.depth_pid(self.curr_depth)

def callback_yaw(self, yaw):
    self.curr_yaw = yaw

def callback_pressure(self, pressure):
    self.curr_depth = pressure_val_to_depth(depth) 
    if self.zero_depth is none:
        self.zero_depth = self.curr_depth
