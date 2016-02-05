#/usr/bin/env python
# Script which goes with robot_2d_description package.
# Load simple 'robot' point-cylinder and cylinder-obstacle to test methods.


from hpp.corbaserver.ur2_robot import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver import Client
import sys

robot = Robot ('ur2_robot')
ps = ProblemSolver (robot)
cl = robot.client
cl.obstacle.loadObstacleModel('ur2_description','cylinder_obstacle','')

#ps.selectPathValidation ("Dichotomy", 0.)
#ps.selectPathPlanner ("VisibilityPrmPlanner")

# alpha_init = 0.2
#q1 = [-0.2, 1.6, -0.4, -1.35, -0.1];
#q2 = [-0.2, 0.4, 0.5, -1.2, -0.4] # two cylinders
#q2 = [-0.2, 0.4, 0.5, -1.35, -0.1] # one cylinder
q1 = [1.6, -0.4, -1.35, -0.1]; q2 = [0.4, 0.5, -1.35, -0.1]; # fixed base
cl.problem.setInitialConfig (q1); cl.problem.addGoalConfig (q2)

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

""" alpha=0.2, ORTH-CONSTR, RELEASE, RRT+DICHO, srand(3000), without prune
Gain (GB): 
average: 46.8101249638
SD: 10.3516299689

Gain (RS): 
average: 55.526270758
SD: 12.3240594603

Gain (PRS): 
average: 63.4773289822
SD: 14.2670102257

Optim comptutation time (GB): 
average: 0.03955168356
SD: 0.0643217971587

alpha=0.2, ORTH-CONSTR, RELEASE, RRT+DICHO, srand(3000), with prune  (alpha=0.3 worse)
Gain (GB): 
average: 47.2524728557
SD: 10.0391488834

Gain (RS): 
average: 58.3331963467
SD: 12.5950218928

Gain (PRS): 
average: 61.5969886537
SD: 11.2929624341

Optim comptutation time (GB): 
average: 0.00420259412
SD: 0.00268984768009

alpha=0.2, ORTH-CONSTR, RELEASE, RRT+DICHO, srand(3000),  (alpha=0.3 worse)  (sure that RRT was used here) 
Gain (GB): 
Number of data: 50
average: 46.4393139685
SD: 11.1000371227

Gain (RS): 
Number of data: 50
average: 53.0442461084
SD: 12.7697170153

Gain (PRS): 
Number of data: 50
average: 61.2786622115
SD: 13.0509990261

Optim comptutation time (GB): 
Number of data: 50
average: 0.02899120384
SD: 0.0307452204443

alpha=0.2, ORTH-CONSTR, RELEASE, RRT+DISRC, srand,  (alpha=0.3 worse)
Gain (GB): 
Number of data: 50
average: 44.7378181103
SD: 9.21727996627

Gain (RS): 
Number of data: 50
average: 53.6822812727
SD: 12.6354136453

Gain (PRS): 
Number of data: 50
average: 56.5525050406
SD: 15.3991479598

Optim comptutation time (GB): 
Number of data: 50
average: 0.02800804928
SD: 0.0190921464053
"""
