#/usr/bin/env python

import time
import sys
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot

robot = Robot ('pr2')
robot.setJointBounds ("base_joint_xy", [-4, -3, -5, -3])
ps = ProblemSolver (robot)
cl = robot.client
ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "") # environment

q_init = robot.getCurrentConfig ()
q_goal = q_init [::]
q_init [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['torso_lift_joint']
q_init [rank] = 0.2

q_goal [0:2] = [-3.2, -4]
rank = robot.rankInConfiguration ['l_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['l_elbow_flex_joint']
q_goal [rank] = -0.5
rank = robot.rankInConfiguration ['r_shoulder_lift_joint']
q_goal [rank] = 0.5
rank = robot.rankInConfiguration ['r_elbow_flex_joint']
q_goal [rank] = -0.5

ps.setInitialConfig (q_init); ps.addGoalConfig (q_goal)

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

