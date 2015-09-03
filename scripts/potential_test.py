#/usr/bin/env python
# Script which goes with potential_description package.
# Load planar 'robot' cylinder and concave obstacles.

from hpp.corbaserver.potential import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import sys

kRange = 5
robot = Robot ('potential')
robot.setJointBounds('base_joint_xy', [-kRange, kRange, -kRange, kRange])
ps = ProblemSolver (robot)
cl = robot.client

# q = [x, y, theta] # (z not considered since planar)
q1 = [-4, 4, 1, 0]; q2 = [4, -4, 1, 0] # obstS 1 # if UNbounded rotation
#q1 = [-4, 4, 0]; q2 = [4, -4, 0] # obstS 1 # if bounded rotation
#q1 = [2.4, -4.6, 1.0, 0.0]; q2 = [-0.4, 4.6, 1.0, 0.0] # obstS 2

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
cl.obstacle.loadObstacleModel('potential_description','obstacles_concaves','')


imax=30
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathLength = ps.pathLength (ps.numberPaths()-1)
    
    ps.addPathOptimizer('RandomShortcut')
    optimTimeRS = cl.problem.optimizePath(ps.numberPaths()-1)
    pathLengthRS = ps.pathLength (ps.numberPaths()-1)
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('GradientBased')
    optimTimeGB = ps.optimizePath(ps.numberPaths()-2)
    pathLengthGB = ps.pathLength (ps.numberPaths()-1)
    ps.clearRoadmap ()
    ps.clearPathOptimizers()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Cost of non-optimized path: '+str(initialPathLength)+'\n')
    f.write('Cost of optimized path (GB): '+str(pathLengthGB)+'\n')
    f.write('Cost of optimized path (RS): '+str(pathLengthRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')

f.close()

from parseRuns import main
main()
