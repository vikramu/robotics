# Firstly import robot class of the controller module
from controller import Robot

# Now create the Robot instance.
robot = Robot()

# get the time step of the current world
timestep = 64

# set the Max Speed of the motor
max_speed = 6.28


lm = robot.getMotor("left_motor")
rm = robot.getMotor("right_motor")

lm.setPosition(float('inf'))
lm.setVelocity(0.0)

rm.setPosition(float('inf'))
rm.setVelocity(0.0)

# Now We have Sensors on the robocart 
# IR_5 is skipped from naming just for the sake that grouping will be easy [1-4] [6-9]
sensors=[]
names=["IR_1","IR_2","IR_3","IR_4","IR_6","IR_7","IR_8","IR_9"]
reading=[0,0,0,0,0,0,0,0]


# Note: 8 sensors Only - range value is 0 - 8
# get readings
for i in range (0,8):
    sensors.append(robot.getDistanceSensor(names[i]))
    sensors[i].enable(timestep)

def getReading():
    for i in range (0,8):
        #print(sensors[i].getValue())
        if int(sensors[i].getValue())>512:
            reading[i] = 1
        else:
            reading[i] = 0
    print(reading)

# PID (proportional integral derivative) controllers use a control loop feedback mechanism 
# This will control process variables and are the most accurate and stable controller.   
previous_error=0.0
kp=10.0
kd=0.0
ki=0.0
Integral=0.0

def PID():
    error=0
    coefficient=[-8000, -6000, -4000, -2000, 2000, 4000, 6000, 8000]
    for i in range(0,8):
        error+=coefficient[i]*reading[i]
        #print(error)
    
    P=kp*error
    I=Integral+(ki*error)
    D=kd*(error-previous_error)
    
    correction=(P+I+D)/1000
    #print(correction)
    l_speed=10+correction
    r_speed=10-correction
    #print (l_speed)
    #print (r_speed)

    if l_speed<0.0 : l_speed=0
    if l_speed>10.0 : l_speed=10.0
    if r_speed<0.0 : r_speed=0
    if r_speed>10.0 : r_speed=10.0
    
    lm.setVelocity(l_speed)
    rm.setVelocity(r_speed)
       
    print(l_speed,r_speed, reading)
    return I,error
    
while (robot.step(timestep) != -1):
    getReading()    
    Integral,previous_error=PID()
    pass
