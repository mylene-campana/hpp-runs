#/usr/bin/env python
# Script which goes with potential_description package.
# Load planar 'robot' cylinder and concave obstacles.

from hpp.corbaserver.potential import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import sys

kRange = 5
robot = Robot ('potential')
robot.setJointBounds('base_joint_xy', [-kRange, kRange, -kRange, kRange])
ps = ProblemSolver (robot)
cl = robot.client

# q = [x, y, theta] # (z not considered since planar)
q1 = [-4, 4, 1, 0]; q2 = [4, -4, 1, 0] # obstS 1 # if UNbounded rotation
#q1 = [-4, 4, 0]; q2 = [4, -4, 0] # obstS 1 # if bounded rotation
#q1 = [2.4, -4.6, 1.0, 0.0]; q2 = [-0.4, 4.6, 1.0, 0.0] # obstS 2

ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

ps.setInitialConfig (q1); ps.addGoalConfig (q2)
cl.obstacle.loadObstacleModel('potential_description','obstacles_concaves','')


imax=50; jmax = 50
f = open('results.txt','a')

for i in range(0, imax):
    print i
    ps.solve ()
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (initialPathNumber)
    print "solve finished"
    
    ps.addPathOptimizer("Prune")
    ps.optimizePath (initialPathNumber)
    initialPathNumber = ps.numberPaths()-1
    
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
        cl.problem.optimizePath(initialPathNumber)
        val = (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
        gainRS = gainRS + val
    gainRS = gainRS/jmax
    print "RS finished"
    print str(gainRS)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('PartialShortcut')
    gainPRS = 0
    for j in range(0, jmax):
        cl.problem.optimizePath(initialPathNumber)
        val = (1-(initialPathLength - ps.pathLength (ps.numberPaths()-1))/initialPathLength)*100
        gainPRS = gainPRS + val
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

""" alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand(3000), without Prune
Gain (GB): 
average: 78.2773545989
SD: 11.3248207689

Gain (RS): 
average: 67.9895186485
SD: 17.954407198

Gain (PRS): 
average: 81.7992908214
SD: 12.1962286277

Optim comptutation time (GB): 
average: 0.01193688574
SD: 0.0160522548346

alpha=0.2, ORTH-CONSTR, RELEASE, PRM+DICHO, srand(3000), WITH Prune
Gain (GB): 
average: 78.0061609518
SD: 13.9973814785

Gain (RS): 
average: 71.1366727406
SD: 14.5580765321

Gain (PRS): 
average: 82.3271507027
SD: 12.2434957811

Optim comptutation time (GB): 
average: 0.00963156086
SD: 0.00845595116265
"""
