#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.

from hpp.corbaserver.robot_2d import Robot
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


robot = Robot ('robot_2d')
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('robot_2d_description','cylinder_obstacle','')


# q = [x, y] # limits in URDF file
q1 = [-2, 0]; q2 = [-1, 1]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-1, 1]; q2 = [-1.2, 1.8]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-1.2, 1.8]; q2 = [-0.2, 1.2]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-0.2, 1.2]; q2 = [0.5, 1.9]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [0.5, 1.9]; q2 = [2, 1.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [2, 1.5]; q2 = [1, 0.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [1, 0.5]; q2 = [2, 0]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()
cl.problem.resetGoalConfigs ()
q1 = [-2, 0]; q2 = [2, 0]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2); cl.problem.solve ()

begin=time.time()
ps.optimizePath(7)
end=time.time()
optimTime = end - begin

# Write important results #
f = open('results.txt','a')
f.write('Try number: '+str(nPlot)+'\n')
f.write('Number of nodes: '+str(len(ps.nodes ()))+'\n')
f.write('Duration of non-optimized path: '+str(ps.pathLength(0))+'\n')
f.write('Duration of optimized path: '+str(ps.pathLength(1))+'\n')
f.write('Solving duration: '+str(0)+'\n') # initial path given by hand
f.write('Optim duration: '+str(optimTime)+'\n')
f.close()


