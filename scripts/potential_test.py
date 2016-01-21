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
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (ps.numberPaths()-1)
    
    ps.addPathOptimizer('RandomShortcut')
    optimTimeRS = cl.problem.optimizePath(initialPathNumber)
    pathLengthRS = ps.pathLength (ps.numberPaths()-1)
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('PartialShortcut')
    optimTimePRS = cl.problem.optimizePath(initialPathNumber)
    pathLengthPRS = ps.pathLength (ps.numberPaths()-1)
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('GradientBased')
    optimTimeGB = ps.optimizePath(initialPathNumber)
    pathLengthGB = ps.pathLength (ps.numberPaths()-1)
    ps.clearRoadmap ()
    ps.clearPathOptimizers()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Cost of non-optimized path: '+str(initialPathLength)+'\n')
    f.write('Cost of optimized path (GB): '+str(pathLengthGB)+'\n')
    f.write('Cost of optimized path (RS): '+str(pathLengthRS)+'\n')
    f.write('Cost of optimized path (PRS): '+str(pathLengthPRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')
    f.write('Optim comptutation time (PRS): '+str(optimTimePRS)+'\n')

f.close()

from parseRuns import main
main()

""" alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand(50)
Cost of non-optimized path parsing: 
average: 21.9068901407
SD: 5.00390135681

Cost of optimized path (GB): 
average: 16.9749945327
SD: 4.14130012262

Cost of optimized path (RS): 
average: 17.7613544782
SD: 5.03357765459

Cost of optimized path (PRS): 
average: 21.7273923394
SD: 5.01352723362

Optim comptutation time (GB): 
average: 0.00929335853333
SD: 0.00832927171301

Optim comptutation time (RS): 
average: 0.0018937526
SD: 0.00126126273914

Optim comptutation time (PRS): 
average: 0.000335699233333
SD: 8.34918666872e-05
"""
