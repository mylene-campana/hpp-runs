#/usr/bin/env python

from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.pr2 import Robot
from parseRuns import main, computeBaseMotion

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

imax=30
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathLength = ps.pathLength (ps.numberPaths()-1)
    L0 = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    
    ps.addPathOptimizer('RandomShortcut')
    optimTimeRS = cl.problem.optimizePath(ps.numberPaths()-1)
    pathLengthRS = ps.pathLength (ps.numberPaths()-1)
    LRS = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('GradientBased')
    optimTimeGB = ps.optimizePath(ps.numberPaths()-2)
    pathLengthGB = ps.pathLength (ps.numberPaths()-1)
    LGB = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    ps.clearRoadmap ()
    ps.clearPathOptimizers()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Cost of non-optimized path: '+str(initialPathLength)+'\n')
    f.write('Cost of optimized path (GB): '+str(pathLengthGB)+'\n')
    f.write('Cost of optimized path (RS): '+str(pathLengthRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')
    f.write('base motion distance: '+str(L0)+'\n')
    f.write('base motion distance (GB): '+str(LGB)+'\n')
    f.write('base motion distance (RS): '+str(LRS)+'\n')

f.close()
main()
