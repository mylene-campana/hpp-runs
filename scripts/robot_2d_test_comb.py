#/usr/bin/env python
# Script which goes with comb_description package.
# Load simple 'robot' point-cylinder and comb-obstacle to test methods.

from hpp.corbaserver.robot_2d_comb import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys

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

