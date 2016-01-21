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

""" alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand
Cost of non-optimized path parsing: 
average: 15.3180109816
SD: 4.77461274688

Cost of optimized path (GB): 
average: 4.52667169755
SD: 2.53736177967

Cost of optimized path (RS): 
average: 5.75185955211
SD: 1.74096207642

Cost of optimized path (PRS): 
average: 12.2249057666
SD: 4.78394833669

Optim comptutation time (GB): 
average: 19.1569382099
SD: 14.6499042748

Optim comptutation time (RS): 
average: 9.88183920165
SD: 9.19352482248

Optim comptutation time (PRS): 
average: 23.4849299963
SD: 9.19586853178
"""


