#/usr/bin/env python

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
from parseRuns import computeBaseMotion

robot = Robot ('pr2')
robot.setJointBounds ("base_joint_xy", [-5, 5, -5, 5])
ps = ProblemSolver (robot)
cl = robot.client

q1 = robot.getCurrentConfig ()
q1 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = -0.02 # un-spread arms
q1 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = 0.02
q1 [robot.rankInConfiguration ['l_shoulder_lift_joint']] = -0.16 # up/down arms
q1 [robot.rankInConfiguration ['r_shoulder_lift_joint']] = 0.14
q1 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # turn arm on itself
q1 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.57
q1 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.85 # bend elbow
q1 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.85
q1 [robot.rankInConfiguration ['l_wrist_flex_joint']] = -0.3 # bend wrist
q1 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.3

q2=q1[::]
q2 [robot.rankInConfiguration ['l_shoulder_lift_joint']] = 0.14 # up/down arms
q2 [robot.rankInConfiguration ['r_shoulder_lift_joint']] = -0.16

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

imax=50; jmax = 50
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (initialPathNumber)
    L0 = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    print "solve finished"
    
    #ps.addPathOptimizer("Prune")
    #ps.optimizePath (initialPathNumber)
    #initialPathNumber = ps.numberPaths()-1
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('GradientBased')
    cl.problem.setAlphaInit (0.2)
    ps.optimizePath(initialPathNumber)
    print "GB finished"
    optimTimeGB = cl.problem.getTimeGB ()
    gainGB = (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
    LGB = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    print str(optimTimeGB)
    print str(gainGB)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('RandomShortcut')
    gainRS = 0; LRS = 0
    for j in range(0, jmax):
        cl.problem.optimizePath(initialPathNumber)
        gainRS = gainRS + (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
        LRS = LRS + computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    gainRS = gainRS/jmax; LRS = LRS/jmax
    print "RS finished"
    print str(gainRS)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('PartialShortcut')
    gainPRS = 0; LPRS = 0
    for j in range(0, jmax):
        cl.problem.optimizePath(initialPathNumber)
        gainPRS = gainPRS + (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
        LPRS = LPRS + computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    gainPRS = gainPRS/jmax; LPRS = LPRS/jmax
    print "PRS finished"
    print str(gainPRS)+'\n'
    
    ps.clearRoadmap (); ps.clearPathOptimizers()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Gain (GB): '+str(gainGB)+'\n')
    f.write('Gain (RS): '+str(gainRS)+'\n')
    f.write('Gain (PRS): '+str(gainPRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')
    f.write('base motion distance: '+str(L0)+'\n')
    f.write('base motion distance (GB): '+str(LGB)+'\n')
    f.write('base motion distance (PRS): '+str(LPRS)+'\n')
    f.write('base motion distance (RS): '+str(LRS)+'\n')

f.close()

from parseRunsSimple import main
main()

""" alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand(3000) puis srand (car plante au 19e solve)
Gain (GB): 
average: 19.854105324
SD: 9.31098550644

Gain (RS): 
average: 43.2114712961
SD: 17.8209428057

Gain (PRS): 
average: 95.1633167133
SD: 3.85201175523

Optim comptutation time (GB): 
average: 0.88212725522
SD: 1.06926025587
"""
