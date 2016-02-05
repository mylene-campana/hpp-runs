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

#ps.selectPathValidation ("Dichotomy", 0.)  # en fait, dicho etait actif avant ...
ps.selectPathPlanner ("VisibilityPrmPlanner")

imax=50; jmax = 50
f = open('results.txt','a')

for i in range(0, imax):
    print i
    solveTime = ps.solve ()
    initialPathNumber = ps.numberPaths()-1
    initialPathLength = ps.pathLength (initialPathNumber)
    print "solve finished"
    print "solveTime= "+str(solveTime)
    print "initialPathLength= "+str(initialPathLength)
    
    #ps.clearPathOptimizers(); ps.addPathOptimizer("Prune")
    #ps.optimizePath (initialPathNumber)
    #initialPathNumber = ps.numberPaths()-1
    #initialPathLength = ps.pathLength (initialPathNumber)
    
    ps.clearPathOptimizers(); ps.addPathOptimizer('GradientBased')
    cl.problem.setAlphaInit (0.1)
    print "start GB optimization"
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
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Gain (GB): '+str(gainGB)+'\n')
    f.write('Gain (RS): '+str(gainRS)+'\n')
    f.write('Gain (PRS): '+str(gainPRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')
    
    # Give a 2nd try to GB with different alpha_init
    ps.clearPathOptimizers(); ps.addPathOptimizer('GradientBased')
    cl.problem.setAlphaInit (0.05)
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
    f.write('Try number: '+str(i+1)+' with 2nd alphaInit'+'\n')
    f.write('Gain (GB): '+str(gainGB)+'\n')
    f.write('Gain (RS): '+str(gainRS)+'\n')
    f.write('Gain (PRS): '+str(gainPRS)+'\n')
    f.write('Optim comptutation time (GB): '+str(optimTimeGB)+'\n')

f.close()

from parseRunsSimple import main
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

alpha=0.05, ORTH-CONSTR2, RELEASE, PRM+Discr, srand
Gain (GB): 
Number of data: 50
average: 53.0559282501
SD: 11.1577549108

Gain (RS): 
Number of data: 50
average: 39.4277555072
SD: 12.2038243776

Gain (PRS): 
Number of data: 50
average: 46.0697522754
SD: 14.3927107186

Optim comptutation time (GB): 
Number of data: 50
average: 0.74200265194
SD: 0.651077892797
"""
