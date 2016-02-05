#/usr/bin/env python
# Script which goes with ur_description package.
# Load 6-DoF arm robot to test methods.

from __future__ import division
from hpp.corbaserver.baxter import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import sys

robot = Robot ('robot')
ps = ProblemSolver (robot)
cl = robot.client


q1 = robot.getCurrentConfig ();
q1 [robot.rankInConfiguration ['torso_t0']] = 0
q1 [robot.rankInConfiguration ['head_pan']] = 0.2
q1 [robot.rankInConfiguration ['left_s0']] = -0.9 # turn +left/right-
q1 [robot.rankInConfiguration ['left_s1']] = -0.32 # +up/down-
q1 [robot.rankInConfiguration ['left_e0']] = 0
q1 [robot.rankInConfiguration ['left_e1']] = 0.68
q1 [robot.rankInConfiguration ['left_w0']] = 0
q1 [robot.rankInConfiguration ['left_w1']] = 0.4 # -up/down+
q1 [robot.rankInConfiguration ['right_s0']] = 0.84
q1 [robot.rankInConfiguration ['right_s1']] = 0.04
q1 [robot.rankInConfiguration ['right_e0']] = 0
q1 [robot.rankInConfiguration ['right_e1']] = -0.05
q1 [robot.rankInConfiguration ['right_w0']] = 0.18
q1 [robot.rankInConfiguration ['right_w1']] = 0.66

q2=q1[::]
q2 [robot.rankInConfiguration ['torso_t0']] = -1.57
q2 [robot.rankInConfiguration ['left_s0']] = -0.42
q2 [robot.rankInConfiguration ['left_s1']] = -0.41
q2 [robot.rankInConfiguration ['left_e0']] = -1.2
q2 [robot.rankInConfiguration ['left_e1']] = 0.62
q2 [robot.rankInConfiguration ['left_w0']] = -0.1
q2 [robot.rankInConfiguration ['left_w1']] = 0.6
q2 [robot.rankInConfiguration ['right_s0']] = 0.42
q2 [robot.rankInConfiguration ['right_s1']] = 0.56
q2 [robot.rankInConfiguration ['right_e0']] = 0
q2 [robot.rankInConfiguration ['right_e1']] = 0
q2 [robot.rankInConfiguration ['right_w0']] = 0.18
q2 [robot.rankInConfiguration ['right_w1']] = 0.49

#ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

# Load obstacles in HPP #
cl.obstacle.loadObstacleModel('room_description','room','')

imax=50; jmax = 50
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (initialPathNumber)
    print "nb waypoints "+str(len(ps.getWaypoints(initialPathNumber)))
    print "solve finished"
    
    #ps.addPathOptimizer("Prune") # 'cause RRT has too many node, so big computation time ...
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

""" alpha=0.2, ORTH-CONSTR V2, RELEASE,PRUNE -_- PRM+DISCR, srand  (DICHO -> big cmpt time .....  like 114s average !)
Number of data: 50
average: 36.4689283629
SD: 10.5974852082

Gain (RS): 
Number of data: 50
average: 45.2360679768
SD: 16.6709887491

Gain (PRS): 
Number of data: 50
average: 79.8785864758
SD: 6.25503947044

Optim comptutation time (GB): 
Number of data: 50
average: 18.7711023962
SD: 11.2740211683

"""

