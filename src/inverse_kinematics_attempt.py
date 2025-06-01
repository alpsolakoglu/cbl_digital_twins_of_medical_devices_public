import pybullet as p
import pybullet_data
import time
import math
import datetime

# Start PyBullet with GUI
physics_client = p.connect(p.SHARED_MEMORY)
if physics_client < 0:
    p.connect(p.GUI)

# Setup simulation
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.8)
time_step = 1.0 / 240.0
p.setTimeStep(time_step)

# Load the floor
p.loadURDF("plane.urdf")
arm_id = p.loadURDF("/src/urdf/4dof.urdf")
p.resetBasePositionAndOrientation(arm_id, [0, 0, 0], [0, 0, 0, 1])
arm_end_effector_index = 3

# Exclude the fixed joint
num_non_fixed_joints = p.getNumJoints(arm_id) - 1  
if (num_non_fixed_joints != 4):
  exit()

# Add user sliders
r_axis_slider = p.addUserDebugParameter(" R-Axis Angle (deg)", 0, 360, 0)
a_axis_slider = p.addUserDebugParameter(" A-Axis Angle (deg)", -90, 90, 0)
b_axis_slider = p.addUserDebugParameter(" B-Axis Angle (deg)", 0, 160, 0)
c_axis_slider = p.addUserDebugParameter(" C-Axis Angle (deg)", 0, 160, 0)

# Joint indices
R_AXIS_JOINT_IDX = 0

# Fixed joint index (not used in control)
FIXED_JOINT_IDX = 1

A_AXIS_JOINT_IDX = 2
B_AXIS_JOINT_IDX = 3
C_AXIS_JOINT_IDX = 4


#lower limits for null space
ll = [0, -math.pi / 4, 0, 0]
#upper limits for null space
ul = [math.pi, math.pi / 4, 2.8, 2.8]
#joint ranges for null space
jr = [math.pi,  math.pi / 2, 2.8, 2.8]
#restposes for null space
rp = [0, 0, 0, 0]
#joint damping coefficents
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(num_non_fixed_joints):
  p.resetJointState(arm_id, i, rp[i])

p.setGravity(0, 0, 0)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = 0
useNullSpace = 1

useOrientation = 1
#If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
#This can be used to test the IK result accuracy.
useSimulation = 1
useRealTimeSimulation = 0
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
#trailDuration is duration (in seconds) after debug lines will be removed automatically
#use 0 for no-removal
trailDuration = 15

i=0
while 1:
  i+=1
  #p.getCameraImage(320,
  #                 200,
  #                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
  #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)
  if (useRealTimeSimulation):
    dt = datetime.now()
    t = (dt.second / 60.) * 2. * math.pi
  else:
    t = t + 0.01

  if (useSimulation and useRealTimeSimulation == 0):
    p.stepSimulation()

  for i in range(1):
    pos = [0, -0.5 + math.cos(t / 100) / 2  / 2, 0.5]
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    if (useNullSpace == 1):
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(arm_id, arm_end_effector_index, pos, orn, ll, ul,
                                                  jr, rp)
      else:
        jointPoses = p.calculateInverseKinematics(arm_id,
                                                  arm_end_effector_index,
                                                  pos,
                                                  lowerLimits=ll,
                                                  upperLimits=ul,
                                                  jointRanges=jr,
                                                  restPoses=rp)
    else:
      if (useOrientation == 1):
        jointPoses = p.calculateInverseKinematics(arm_id,
                                                  arm_end_effector_index,
                                                  pos,
                                                  orn,
                                                  jointDamping=jd,
                                                  solver=ikSolver,
                                                  maxNumIterations=100,
                                                  residualThreshold=.01)
      else:
        jointPoses = p.calculateInverseKinematics(arm_id,
                                                  arm_end_effector_index,
                                                  pos,
                                                  solver=ikSolver)

    if (useSimulation):
      for i in range(num_non_fixed_joints):
        p.setJointMotorControl2(bodyIndex=arm_id,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointPoses[i],
                                targetVelocity=0,
                                force=100,
                                positionGain=0.03,
                                velocityGain=1)
    else:
      #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
      for i in range(num_non_fixed_joints):
        p.resetJointState(arm_id, i, jointPoses[i])

  ls = p.getLinkState(arm_id, arm_end_effector_index)
  if (hasPrevPose):
    p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
    p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
  prevPose = pos
  prevPose1 = ls[4]
  hasPrevPose = 1