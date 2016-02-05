#/usr/bin/env python

import sys
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot

robot = Robot ('pr2')
robot.setJointBounds ("base_joint_xy", [-4.5, -2.8, -7, -3])
ps = ProblemSolver (robot)
cl = robot.client
ps.loadObstacleFromUrdf ("iai_maps", "kitchen_area", "")
ps.loadObstacleFromUrdf ("iai_maps", "floor", "")
ps.loadObstacleFromUrdf ("iai_maps", "chair", "")
ps.loadObstacleFromUrdf ("iai_maps", "set", "")
ps.loadObstacleFromUrdf ("iai_maps", "flower-vase", "")

q1 = robot.getCurrentConfig ()
q2 = q1 [::]
q1 [0:2] = [-3.36, -5.8]
rank = robot.rankInConfiguration ['torso_lift_joint']
q1 [rank] = 0.2
q1 [robot.rankInConfiguration ['torso_lift_joint']] = 0.13
q1 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = 0.5 # spread arms
q1 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = -0.5
q1 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # turn arm on itself
q1 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.57
q1 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.49 # bend elbow
q1 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -0.50
q1 [robot.rankInConfiguration ['l_wrist_flex_joint']] = -0.2 # bend wrist
q1 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.2
q1 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0.09 # open gripper
q1 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0.12
q1 [robot.rankInConfiguration ['r_gripper_r_finger_joint']] = 0.13
q1 [robot.rankInConfiguration ['r_gripper_l_finger_joint']] = 0.095

q2 [0:2] = [-3.9, -3.8]
q2 [2:4] = [-1, 0]
q2 [robot.rankInConfiguration ['l_shoulder_pan_joint']] = 0.2 # spread arms
q2 [robot.rankInConfiguration ['r_shoulder_pan_joint']] = -0.7
q2 [robot.rankInConfiguration ['l_shoulder_lift_joint']] = 0.9 # up/down arms
q2 [robot.rankInConfiguration ['r_shoulder_lift_joint']] = -0.1
q2 [robot.rankInConfiguration ['l_upper_arm_roll_joint']] = 1.57 # turn arm on itself
q2 [robot.rankInConfiguration ['r_upper_arm_roll_joint']] = -1.2
q2 [robot.rankInConfiguration ['l_elbow_flex_joint']] = -0.3 # plier coude
q2 [robot.rankInConfiguration ['r_elbow_flex_joint']] = -1.15
q2 [robot.rankInConfiguration ['l_wrist_flex_joint']] = 0 # bend wrist
q2 [robot.rankInConfiguration ['r_wrist_flex_joint']] = -0.3
q2 [robot.rankInConfiguration ['r_wrist_roll_joint']] = 0 # turn wrist [CONTINUOUS]
q2 [robot.rankInConfiguration ['r_wrist_roll_joint']+1] = 1 # turn wrist poignet
q2 [robot.rankInConfiguration ['l_gripper_r_finger_joint']] = 0 # open gripper
q2 [robot.rankInConfiguration ['l_gripper_l_finger_joint']] = 0

#ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

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
    cl.problem.setAlphaInit (0.1)
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

""" alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand

"""
