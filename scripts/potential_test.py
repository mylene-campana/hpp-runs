#/usr/bin/env python
# Script which goes with potential_description package.
# Load planar 'robot' cylinder and concave obstacles.

from hpp.corbaserver.potential import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
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

kRange = 5
robot = Robot ('potential')
robot.setJointBounds('base_joint_xy', [-kRange, kRange, -kRange, kRange])
ps = ProblemSolver (robot)
cl = robot.client

# q = [x, y, theta] # (z not considered since planar)
q1 = [-4, 4, 1, 0]; q2 = [4, -4, 1, 0] # obstS 1
#q1 = [2.4, -4.6, 1.0, 0.0]; q2 = [-0.4, 4.6, 1.0, 0.0] # obstS 2

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
cl.obstacle.loadObstacleModel('potential_description','obstacles_concaves','')

#ps.createOrientationConstraint ("orConstraint", "base_joint_rz", "", [0.7071067812,0,0,0.7071067812], [0,0,1]) # OK
#T = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,1]]
#ps.setNumericalConstraints ("constraints", ["orConstraint"])

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
