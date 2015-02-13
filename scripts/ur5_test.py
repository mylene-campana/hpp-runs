#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time
import sys

# Parse arguments : number and path #
print 'Argument List: ', str(sys.argv)
if len(sys.argv) != 3: # Warning: fileName is counted by default
  sys.exit("Not enough args")
arg_one = str(sys.argv[1])
nPlot=int(sys.argv[1]) 
arg_two = str(sys.argv[2])
print 'Arguments parsed: '+arg_one + ', '+arg_two

robot = Robot ('ur5')
cl = robot.client
#robot.setRootJointPosition([0,0,1.2,1,0,0,0]) # only in HPP, not displayed

# q = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 6 DoF#
q1 = [0, -1.57, 1.57, 0, 0, 0]; q2 = [0.2, -1.57, -1.8, 0, 0.8, 0]

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacles','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')
cl.obstacle.loadObstacleModel('ur_description','wall','') # wall with hole

begin=time.time()
ps.solve ()
end=time.time()
solveTime = end - begin

begin=time.time()
ps.optimizePath(0)
end=time.time()
optimTime = end - begin

# Write important results #
f = open('results.txt','a')
f.write('Try number: '+str(nPlot)+'\n')
f.write('Number of nodes: '+str(len(ps.nodes ()))+'\n')
f.write('Duration of non-optimized path: '+str(ps.pathLength(0))+'\n')
f.write('Duration of optimized path: '+str(ps.pathLength(1))+'\n')
f.write('Solving duration: '+str(solveTime)+'\n')
f.write('Optim duration: '+str(optimTime)+'\n')
f.close()

