#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import sys

robot = Robot ('ur5')
ps =ProblemSolver (robot)
cl = robot.client
#robot.setRootJointPosition([0,0,1.2,1,0,0,0]) # only in HPP, not displayed

# q = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 6 DoF#
q1 = [0, -1.57, 1.57, 0, 0, 0]; q2 = [0.2, -1.57, -1.8, 0, 0.8, 0]

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacles','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')
cl.obstacle.loadObstacleModel('ur_description','wall','') # wall with hole

imax=50
f = open('results.txt','a')

for i in range(0, imax):
    solveTime = ps.solve ()
    
    ps.addPathOptimizer('GradientBased')
    ps.optimizePath(i*3)
    optimTimeGB = cl.problem.getComputationTime ()
    iterNbGB = cl.problem.getIterationNumber ()
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('RandomShortcut')
    cl.problem.optimizePathLength(i*3, ps.pathLength(i*3+1))
    optimTimeRS = cl.problem.getComputationTime ()
    iterNbRS = cl.problem.getIterationNumber ()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Cost of non-optimized path: '+str(ps.pathLength(i))+'\n')
    f.write('Cost of optimized path (GB): '+str(ps.pathLength(i*3+1))+'\n')
    f.write('Cost of optimized path (RS): '+str(ps.pathLength(i*3+2))+'\n')
    f.write('Solving comptutation time: '+str(solveTime)+'\n')
    f.write('Optim comptutation time (BG): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')
    f.write('Nb waypoints: '+str(len(ps.getWaypoints (0)))+'\n')
    f.write('Nb iterations (GB): '+str(iterNbGB)+'\n')
    f.write('Nb iterations (RS): '+str(iterNbRS)+'\n')

f.close()

