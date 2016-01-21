#/usr/bin/env python
# Script which goes with puzzle_description package.
# RViz command : roslaunch puzzle_description puzzle.launch
# Easy way to test planning algo (no internal DoF) on SO3 joint.

from hpp.corbaserver.puzzle import Robot
from hpp.corbaserver import Client
from hpp.corbaserver import ProblemSolver
import sys

robot = Robot ('puzzle') # object5
robot.setJointBounds('base_joint_xyz', [-0.9, 0.9, -0.9, 0.9, -1.1, 1.1])
ps = ProblemSolver (robot)
cl = robot.client
#cl.obstacle.loadObstacleModel('puzzle_description','decor_very_easy','')
cl.obstacle.loadObstacleModel('puzzle_description','decor_easy','')

q1 = [0.0, 0.0, 0.8, 1.0, 0.0, 0.0, 0.0]; q2 = [0.0, 0.0, -0.8, 1.0, 0.0, 0.0, 0.0]
#q1 = [0, 0, 0.8, 1, 0, 1, 0, 1, 0]; q2 = [0, 0, -0.8, 1, 0, 1, 0, 1, 0]

ps.setInitialConfig (q1); ps.addGoalConfig (q2)

#ps.selectPathValidation ("Dichotomy", 0.)
ps.selectPathPlanner ("VisibilityPrmPlanner")

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

""" alpha=0.05, ORTH-CONSTR, RELEASE, PRM+Discr, srand
Cost of non-optimized path parsing: 
average: 18.7646157285
SD: 5.04195063411

Cost of optimized path (GB): 
average: 9.65292533288
SD: 2.08958863657

Cost of optimized path (RS): 
average: 10.263019161
SD: 3.07513661638

Cost of optimized path (PRS): 
average: 18.7646157285
SD: 5.04195063411

Optim comptutation time (GB): 
average: 1.02010463943
SD: 0.886868480948

Optim comptutation time (RS): 
average: 0.0090014003
SD: 0.00472489570441

Optim comptutation time (PRS): 
average: 0.0011346852
SD: 0.000342523729984

alpha=0.2, ORTH-CONSTR, RELEASE, PRM+Discr, srand
Cost of non-optimized path parsing: 
average: 18.3568617188
SD: 4.86704389184

Cost of optimized path (GB): 
average: 10.5536945894
SD: 3.062689761

Cost of optimized path (RS): 
average: 9.98165056094
SD: 3.29808601745

Cost of optimized path (PRS): 
average: 18.3568617188
SD: 4.86704389184

Optim comptutation time (GB): 
average: 0.6383026579
SD: 0.512575920581

Optim comptutation time (RS): 
average: 0.0094921172
SD: 0.00654571789655

Optim comptutation time (PRS): 
average: 0.0012983721
SD: 0.000446801421961
"""
