#/usr/bin/env python

from hpp.corbaserver.hrp2 import Robot
from hpp.corbaserver import ProblemSolver
from hpp.corbaserver.wholebody_step.client import Client as WsClient
import time
import sys

# Parse arguments : number and path #
print 'Argument List: ', str(sys.argv)
if len(sys.argv) != 3: # Warning: fileName is counted by default
  sys.exit("Not enough args")
arg_one = str(sys.argv[1])
nPlot=int(sys.argv[1]) 
arg_two = str(sys.argv[2])
print 'Arguments parsed: '+arg_one + ', '+arg_two

#Robot.urdfSuffix = '_capsule'
#Robot.srdfSuffix= '_capsule'

robot = Robot ('hrp2_14')
#robot.setJointBounds ('base_joint_xyz', [-2.5, 2.5, -2.5, 2.5, 0.62, 0.68])
robot.setJointBounds ('base_joint_xyz', [-3, 3, -3, 3, 0, 1])
ps = ProblemSolver (robot)
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

begin=time.time()
ps.solve ()
end=time.time()
solveTime = end - begin

begin=time.time()
ps.optimizePath(0)
end=time.time()
optimTime = end - begin

# Write important results #
f = open('results.txt','a')
f.write('Try number: '+str(nPlot)+'\n')
f.write('Duration of non-optimized path: '+str(ps.pathLength(0))+'\n')
f.write('Duration of optimized path: '+str(ps.pathLength(1))+'\n')
f.write('Solving duration: '+str(solveTime)+'\n')
f.write('Optim duration: '+str(optimTime)+'\n')
f.write('Nb waypoints: '+str(len(ps.getWaypoints (0)))+'\n')
f.write('Nb iterations: '+str(cl.problem.getIterationNumber ())+'\n')
f.close()
