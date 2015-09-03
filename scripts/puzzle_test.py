#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import sys

robot = Robot ('puzzle') # object5
robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1.2, 1.2])
ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')
cl.obstacle.loadObstacleModel('puzzle_description','decor_easy','')

#q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
q1 = [0, 0, 0.8, 1, 0, 1, 0, 1, 0]; q2 = [0, 0, -0.8, 1, 0, 1, 0, 1, 0]

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

imax=30
f = open('results.txt','a')

for i in range(0, imax):
    ps.solve ()
    initialPathLength = ps.pathLength (ps.numberPaths()-1)
    
    ps.addPathOptimizer('RandomShortcut')
    optimTimeRS = cl.problem.optimizePathLength(ps.numberPaths()-1)
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
    f.write('Optim comptutation time (BG): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')

f.close()

from parseRuns import main
main()
