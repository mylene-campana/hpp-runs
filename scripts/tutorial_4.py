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
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (ps.numberPaths()-1)
    L0 = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    
    ps.addPathOptimizer('RandomShortcut')
    optimTimeRS = cl.problem.optimizePath(initialPathNumber)
    pathLengthRS = ps.pathLength (ps.numberPaths()-1)
    LRS = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('PartialShortcut')
    optimTimePRS = cl.problem.optimizePath(initialPathNumber)
    pathLengthPRS = ps.pathLength (ps.numberPaths()-1)
    LPRS = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
    
    ps.clearPathOptimizers()
    ps.addPathOptimizer('GradientBased')
    optimTimeGB = ps.optimizePath(initialPathNumber)
    pathLengthGB = ps.pathLength (ps.numberPaths()-1)
    LGB = computeBaseMotion(ps.getWaypoints (ps.numberPaths()-1))
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
    f.write('base motion distance: '+str(L0)+'\n')
    f.write('base motion distance (GB): '+str(LGB)+'\n')
    f.write('base motion distance (PRS): '+str(LPRS)+'\n')
    f.write('base motion distance (RS): '+str(LRS)+'\n')

f.close()
main()

""" alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand
Cost of non-optimized path parsing: 
average: 12.5535300898
SD: 5.04654567123

Cost of optimized path (GB): 
average: 2.57493668706
SD: 1.88992410863

Cost of optimized path (RS): 
average: 3.59607100758
SD: 1.7304147817

Cost of optimized path (PRS): 
average: 5.31048327573
SD: 1.34949010614

Optim comptutation time (GB): 
average: 1.27918353139
SD: 2.12831164107

Optim comptutation time (RS): 
average: 1.92330562668
SD: 1.28113512768

Optim comptutation time (PRS): 
average: 2.63398091658
SD: 0.754932646746

base motion distance: 
average: 9.56526318827
SD: 4.81092454393

base motion distance (GB): 
average: 2.17863987063e-07
SD: 1.19224212579e-06

base motion distance (RS): 
average: 2.6280511144
SD: 1.56191724153

base motion distance (PRS): 
average: 0.0
SD: 0.0
"""
