#/usr/bin/env python

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

