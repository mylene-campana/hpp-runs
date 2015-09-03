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

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

# alpha_init = 0.2
q1 = [-0.2, 1.6, -0.4, -1.35, -0.1];
q2 = [-0.2, 0.4, 0.5, -1.2, -0.4] # two cylinders
#q2 = [-0.2, 0.4, 0.5, -1.35, -0.1] # one cylinder
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)

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
