#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.


from hpp.corbaserver.ur2_robot import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import sys

robot = Robot ('ur2_robot')
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('ur2_description','cylinder_obstacle','')

q1 = [-0.2, 1.6, -0.4]; q2 = [-0.2, 0.4, 0.5]
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)

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

