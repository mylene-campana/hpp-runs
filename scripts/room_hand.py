#/usr/bin/env python

from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient
import time
import sys

#Robot.urdfSuffix = '_capsule'
#Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
#robot.setJointBounds ('base_joint_xyz', [-2.5, 2.5, -2.5, 2.5, 0.62, 0.68])
robot.setJointBounds ('base_joint_xyz', [-3, 3, -3, 3, 0, 1])
ps = ProblemSolver (robot)
ps.selectPathOptimizer('GradientBased')
cl = robot.client
q0 = robot.getInitialConfig ()

# Add constraints
wcl = WsClient ()
wcl.problem.addStaticStabilityConstraints ("balance", q0, robot.leftAnkle,
                                           robot.rightAnkle)

ps.setNumericalConstraints ("balance", ["balance/relative-com",
                                                "balance/relative-orientation",
                                                "balance/relative-position",
                                                "balance/orientation-left-foot",
                                                "balance/position-left-foot"])

q1 = [0.0, 0.0, 0.65, 1.0, 0., 1., 0., 1, 0.0, 0.0, 0.0, 0.0, 0.0, -0.4, 0, -1.2, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q1)
if res [0]:
    q1proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")


q2 = [0.0, 0.0, 0.65, 1, 0, 1, 0, 1, 0, 0.0, 0.0, 0.0, 0.0, 1.0, 0, -1.4, -1.0, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.261799, -0.17453, 0.0, -0.523599, 0.0, 0.0, 0.174532, -0.174532, 0.174532, -0.174532, 0.174532, -0.174532, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0]

res = ps.applyConstraints (q2)
if res [0]:
    q2proj = res [1]
else:
    raise RuntimeError ("Failed to apply constraint.")

ps.setInitialConfig (q1proj); ps.addGoalConfig (q2proj)


#cl.obstacle.loadObstacleModel('room_description','room','')
#cl.obstacle.loadObstacleModel('room_description','walls','')

imax=50
f = open('results.txt','a')

for i in range(0, imax):
    begin=time.time()
    ps.solve ()
    end=time.time()
    solveTime = end - begin
    
    ps.selectPathOptimizer('GradientBased')
    begin=time.time()
    ps.optimizePath(i*3)
    end=time.time()
    optimTimeGB = end - begin
    iterNbGB = cl.problem.getIterationNumber ()
    
    ps.selectPathOptimizer('RandomShortcut')
    begin=time.time()
    ps.optimizePath(i*3)
    end=time.time()
    optimTimeRS = end - begin
    iterNbRS = cl.problem.getIterationNumber ()
    
    # Write important results #
    f.write('Try number: '+str(i+1)+'\n')
    f.write('Cost of non-optimized path: '+str(ps.pathLength(i))+'\n')
    f.write('Cost of optimized path (GB): '+str(ps.pathLength(i*3+1))+'\n')
    f.write('Cost of optimized path (RS): '+str(ps.pathLength(i*3+2))+'\n')
    f.write('Solving comptutation time: '+str(solveTime)+'\n')
    f.write('Optim comptutation time (BG): '+str(optimTimeGB)+'\n')
    f.write('Optim comptutation time (RS): '+str(optimTimeRS)+'\n')
    f.write('Nb waypoints: '+str(len(ps.getWaypoints (0)))+'\n')
    f.write('Nb iterations (GB): '+str(iterNbGB)+'\n')
    f.write('Nb iterations (RS): '+str(iterNbRS)+'\n')

f.close()

