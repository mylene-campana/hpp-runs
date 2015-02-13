#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.


from hpp.corbaserver.ur2_robot import Robot
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

robot = Robot ('ur2_robot')
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('ur2_description','cylinder_obstacle','')

q1 = [-0.2, 1.6, -0.4]; q2 = [-0.2, 0.4, 0.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)

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

