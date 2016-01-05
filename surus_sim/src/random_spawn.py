#Python script for randomly spawning surus_sim

import subprocess
import random
import math

random.seed()
isLeft = random.randint(0,1)

x   =  0.550
y   = -0.779
z   =  0.280
yaw =  0.0
if (isLeft):
	y = y + 1.779

x = x + random.uniform(-.05,.05)
y = y + random.uniform(-.05,.05)
yaw = yaw + math.pi/2.0*float(random.randint(0,3)) 

print("x = {0}\ny = {1}\nz = {2}\nyaw = {3}".format(x,y,z,yaw))

subprocess.call(["rosrun", "gazebo_ros", "spawn_model", "-sdf", "-param", "robot_description_sdf", "-model", "surus_sim", "-x", str(x), "-y", str(y), "-z", str(z), "-Y", str(yaw)])
