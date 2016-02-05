#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from hpp.corbaserver.ur5_robot import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import sys

robot = Robot ('ur5')
ps =ProblemSolver (robot)
cl = robot.client

# q = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'] 6 DoF#
q1 = [-0.07, -1.3, 1.4, -0.8, 3.14, 0]; q2 = [-0.2, -1.65, -1.6, 0.4, 0.8, 0]

#ps.selectPathValidation ("Dichotomy", 0.)
#ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('ur_description','obstacle_sphere','') # cylinders
cl.obstacle.loadObstacleModel('ur_description','table','')

imax=50; jmax = 50
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (initialPathNumber)
    print "solve finished"
    
    #ps.addPathOptimizer("Prune")
    #ps.optimizePath (initialPathNumber)
    #initialPathNumber = ps.numberPaths()-1
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('GradientBased')
    cl.problem.setAlphaInit (0.3)
    ps.optimizePath(initialPathNumber)
    print "GB finished"
    optimTimeGB = cl.problem.getTimeGB ()
    gainGB = (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
    print str(optimTimeGB)
    print str(gainGB)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('RandomShortcut')
    gainRS = 0
    for j in range(0, jmax):
        cl.problem.optimizePath(initialPathNumber)
        gainRS = gainRS + (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
    gainRS = gainRS/jmax
    print "RS finished"
    print str(gainRS)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('PartialShortcut')
    gainPRS = 0
    for j in range(0, jmax):
        cl.problem.optimizePath(initialPathNumber)
        gainPRS = gainPRS + (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
    gainPRS = gainPRS/jmax
    print "PRS finished"
    print str(gainPRS)+'\n'
    
    ps.clearRoadmap (); ps.clearPathOptimizers()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Gain (GB): '+str(gainGB)+'\n')
    f.write('Gain (RS): '+str(gainRS)+'\n')
    f.write('Gain (PRS): '+str(gainPRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')

f.close()

from parseRunsSimple import main
main()

""" alpha=0.3, ORTH-CONSTR, RELEASE, RRT+DICHO, srand
Gain (GB): 
average: 53.7564985553
SD: 14.6153322391

Gain (RS): 
average: 47.5235637768
SD: 16.2696053537

Gain (PRS): 
average: 67.783740362
SD: 14.6859045976

Optim comptutation time (GB): 
average: 3.18277478902
SD: 2.18309380687

alpha=0.3, ORTH-CONSTR, RELEASE, RRT+DISCR, srand
Gain (GB): 
Number of data: 49
average: 48.1720090571
SD: 14.182824084

Gain (RS): 
Number of data: 49
average: 42.0754674026
SD: 13.8991692445

Gain (PRS): 
Number of data: 49
average: 71.5891367359
SD: 11.233140078

Optim comptutation time (GB): 
Number of data: 49
average: 0.294080723061
SD: 0.219046058236
"""
