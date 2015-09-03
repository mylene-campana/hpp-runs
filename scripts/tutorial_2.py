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

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q_init); ps.addGoalConfig (q_goal)

imax=30
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathLength = ps.pathLength (ps.numberPaths()-1)
    print "solve finished"
    print str(initialPathLength)
    
    ps.addPathOptimizer('RandomShortcut')
    optimTimeRS = cl.problem.optimizePath(ps.numberPaths()-1)
    pathLengthRS = ps.pathLength (ps.numberPaths()-1)
    nPathRS = ps.numberPaths()-1
    print "RS finished"
    print str(optimTimeRS)
    print str(pathLengthRS)
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('GradientBased')
    optimTimeGB = ps.optimizePath(ps.numberPaths()-2)
    #optimTimeGB = ps.optimizePath(nPathRS) # to get RS cost in 'initial cost'
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
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')

f.close()

from parseRuns import main
main()

