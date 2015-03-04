#/usr/bin/env python
# Script which goes with comb_description package.
# Load simple 'robot' point-cylinder and comb-obstacle to test methods.

from hpp.corbaserver.robot_2d_comb import Robot
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


robot = Robot ('robot_2d_comb')
ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('comb_description','comb_obstacle','')
#cl.obstacle.loadObstacleModel('comb_description','broken_comb_obstacle','')
cl.obstacle.loadObstacleModel('comb_description','hard_comb_obstacle','')


# q = [x, y] # limits in URDF file
#q1 = [0, -4.5]; q2 = [0, 4.5]
q1 = [-0.5, -4]; q2 = [-0.5, 4] # hard_comb
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2);


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
f.write('Nb waypoints: '+str(len(ps.getWaypoints (0)))+'\n')
f.write('Nb iterations: '+str(len(cl.problem.getIterationNumber ()))+'\n')
f.close()
