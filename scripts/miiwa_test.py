#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.dlr_miiwa import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import sys

robot = Robot ('dlr_miiwa')
ps = ProblemSolver (robot)
cl = robot.client

robot.setJointBounds('miiwa_joint_x', [-7, 5])
robot.setJointBounds('miiwa_joint_y', [-1.5, 1.5])

q1 = [4, 0.4, 0.8761403383, 0.4820561249, 0.51, -1.1, 0, -0.44, -0.78, 0.44, -0.53, 0, 0, 0, 0]
q2 = [-6.06, 0.465, 0.2721349469, -0.9622590975, -1.22, 1.26, -2.28, -0.84, -1.796, 0.12, -0.53, 0, 0, 0, 0]

#robot.setJointBounds('miiwa_joint_x', [-2, 5]) # easier
#q2 = [-1, 0.4, 0.2721349469, -0.9622590975, -1.22, 1.26, -2.28, -0.84, -1.796, 0.12, -0.53, 0, 0, 0, 0] # easier

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('dlr_miiwa','table','')
cl.obstacle.loadObstacleModel('dlr_miiwa','large_wall','')
cl.obstacle.loadObstacleModel('dlr_miiwa','floor','')

imax=30
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (initialPathNumber)
    print "solve finished"
    print str(initialPathLength)
    
    ps.addPathOptimizer('RandomShortcut')
    optimTimeRS = cl.problem.optimizePath(initialPathNumber)
    pathLengthRS = ps.pathLength (ps.numberPaths()-1)
    print "RS finished"
    print str(optimTimeRS)
    print str(pathLengthRS)
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('PartialShortcut')
    optimTimePRS = cl.problem.optimizePath(initialPathNumber)
    pathLengthPRS = ps.pathLength (ps.numberPaths()-1)
    print "PRS finished"
    print str(optimTimePRS)
    print str(pathLengthPRS)
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('GradientBased')
    optimTimeGB = ps.optimizePath(initialPathNumber)
    print "GB finished"
    pathLengthGB = ps.pathLength (ps.numberPaths()-1)
    print str(optimTimeGB)
    print str(pathLengthGB)
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

