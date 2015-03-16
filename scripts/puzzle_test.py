#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import time
import sys

robot = Robot ('puzzle') # object5
robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1.2, 1.2])
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')

#q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
q1 = [0, 0, 0.8, 1, 0, 1, 0, 1, 0]; q2 = [0, 0, -0.8, 1, 0, 1, 0, 1, 0]

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

imax=50
f = open('results.txt','a')

for i in range(0, imax):
    begin=time.time()
    ps.solve ()
    end=time.time()
    solveTime = end - begin
    
    ps.selectPathOptimizer('GradientBased')
    begin=time.time()
    ps.optimizePath(i*3)
    end=time.time()
    optimTimeGB = end - begin
    iterNbGB = cl.problem.getIterationNumber ()
    
    ps.selectPathOptimizer('RandomShortcut')
    begin=time.time()
    ps.optimizePath(i*3)
    end=time.time()
    optimTimeRS = end - begin
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

