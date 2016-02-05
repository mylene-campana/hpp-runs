#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.ur5_with_gripper import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import sys

robot = Robot ('ur5')
ps =ProblemSolver (robot)
cl = robot.client

# q = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 6 DoF#
q1 = [1.4, -0.75, 1.07, 1.2, 1.57, -0.2]; q2 = [-0.47, -1.3, 2, 0.9, 1.57, -2.7]

#ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','large_table','')
cl.obstacle.loadObstacleModel('ur_description','carton','')
cl.obstacle.loadObstacleModel('ur_description','box','')

imax=50; jmax = 50
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (initialPathNumber)
    print "nb waypoints "+str(len(ps.getWaypoints(initialPathNumber)))
    print "solve finished"
    
    #ps.clearPathOptimizers(); ps.addPathOptimizer("Prune")
    #ps.optimizePath (initialPathNumber)
    #initialPathNumber = ps.numberPaths()-1
    #initialPathLength = ps.pathLength (initialPathNumber)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('GradientBased')
    cl.problem.setAlphaInit (0.2)
    ps.optimizePath(initialPathNumber)
    print "GB finished"
    optimTimeGB = cl.problem.getTimeGB ()
    gainGB = (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
    print str(optimTimeGB)
    print str(gainGB)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('RandomShortcut')
    gainRS = 0
    for j in range(0, jmax):
        print j
        cl.problem.optimizePath(initialPathNumber)
        gainRS = gainRS + (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
    gainRS = gainRS/jmax
    print "RS finished"
    print str(gainRS)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('PartialShortcut')
    gainPRS = 0
    for j in range(0, jmax):
        print j
        cl.problem.optimizePath(initialPathNumber)
        gainPRS = gainPRS + (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
    gainPRS = gainPRS/jmax
    print "PRS finished"
    print str(gainPRS)+'\n'
    
    ps.clearRoadmap ()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Gain (GB): '+str(gainGB)+'\n')
    f.write('Gain (RS): '+str(gainRS)+'\n')
    f.write('Gain (PRS): '+str(gainPRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')

f.close()

from parseRunsSimple import main
main()


""" alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand
Gain (GB): 
Number of data: 50
average: 37.4185012773
SD: 14.2070163762

Gain (RS): 
Number of data: 50
average: 20.8318930653
SD: 14.9240023116

Gain (PRS): 
Number of data: 50
average: 39.6049576253
SD: 19.9721632501

Optim comptutation time (GB): 
Number of data: 50
average: 18.7245152631
SD: 12.9752301053


alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DISCR, srand

"""
