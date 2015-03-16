#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.

from hpp.corbaserver.robot_2d import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import time
import sys

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

imax=30
f = open('results.txt','a')

for i in range(0, imax):
    ps.selectPathOptimizer('GradientBased')
    begin=time.time()
    ps.optimizePath(7)
    end=time.time()
    optimTimeGB = end - begin
    iterNbGB = cl.problem.getIterationNumber ()
    
    ps.selectPathOptimizer('RandomShortcut')
    begin=time.time()
    ps.optimizePath(7)
    end=time.time()
    optimTimeRS = end - begin
    iterNbRS = cl.problem.getIterationNumber ()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Cost of non-optimized path: '+str(ps.pathLength(7))+'\n')
    if i==0:
        f.write('Cost of optimized path (GB): '+str(ps.pathLength(7+1))+'\n')
        f.write('Cost of optimized path (RS): '+str(ps.pathLength(7+2))+'\n')
    else:
        f.write('Cost of optimized path (GB): '+str(ps.pathLength(7+i*2+1))+'\n')
        f.write('Cost of optimized path (RS): '+str(ps.pathLength(7+i*2+2))+'\n')
    
    f.write('Solving comptutation time: '+str(0)+'\n')
    f.write('Optim comptutation time (BG): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')
    f.write('Nb waypoints: '+str(len(ps.getWaypoints (0)))+'\n')
    f.write('Nb iterations (GB): '+str(iterNbGB)+'\n')
    f.write('Nb iterations (RS): '+str(iterNbRS)+'\n')

f.close()

