"""LFR_PID_Code controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
time_step=32
max_speed=4


#motor
left_F_motor=robot.getDevice('left_f_wheel')
right_F_motor=robot.getDevice('right_f_wheel')
left_B_motor=robot.getDevice('left_b_wheel')
right_B_motor=robot.getDevice('right_b_wheel')

left_F_motor.setPosition(float('inf'))
right_F_motor.setPosition(float('inf'))
left_B_motor.setPosition(float('inf'))
right_B_motor.setPosition(float('inf'))

left_F_motor.setVelocity(0.0)
right_F_motor.setVelocity(0.0)
left_B_motor.setVelocity(0.0)
right_B_motor.setVelocity(0.0)


#ir sensor
right_ir=robot.getDevice('ds_left')
right_ir.enable(time_step)

mid_ir=robot.getDevice('ds_mid')
mid_ir.enable(time_step)

left_ir=robot.getDevice('ds_right')
left_ir.enable(time_step)

kp=0.6
ki=0.1
kd=0.1
error=0


base_speed=-3

left_F_motor.setVelocity(base_speed)
right_F_motor.setVelocity(base_speed)
left_B_motor.setVelocity(base_speed)
right_B_motor.setVelocity(base_speed)

last_position=0

while robot.step(time_step) != -1:
   
   
   right_ir_val=right_ir.getValue()
   mid_ir_val=mid_ir.getValue()
   left_ir_val=left_ir.getValue()
   
   if right_ir_val<510:
       right_ir_dg=0
   else:
       right_ir_dg=1

   if left_ir_val<510:
       left_ir_dg=0
   else:
       left_ir_dg=1
       
   if mid_ir_val<510:
       mid_ir_dg=0
   else:
       mid_ir_dg=1    
   
   weight=(-10)*left_ir_dg + 0*mid_ir_dg + 10*right_ir_dg
   
   sum= left_ir_dg+right_ir_dg+mid_ir_dg
   
   
   if sum==0:
       position=last_position
   else:
       position=weight/sum
       last_position=position
   print("left: {} mid: {} right: {}".format(left_ir_val,mid_ir_val,right_ir_val))
   print("sum:{} weight:{}".format(sum,weight))
   print("pos:{} last_pos:{}".format(position,last_position))
   
   motorCorrection=kp*position+ki*position+kd*(position-last_position) 
   
   right_speed=base_speed+motorCorrection
   left_speed=base_speed-motorCorrection
  
  
   left_F_motor.setVelocity(left_speed)
   right_F_motor.setVelocity(right_speed)
   left_B_motor.setVelocity(left_speed)
   right_B_motor.setVelocity(right_speed) 
   pass