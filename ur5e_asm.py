import pybullet as p
import numpy as np

class Ur5e_asm:
  def __init__(self):
    self.reset()

  def buildJointNameToIdDict(self):
    nJoints = p.getNumJoints(self.id)
    self.jointNameToId = {}
    for i in range(nJoints):
      jointInfo = p.getJointInfo(self.id, i)
      self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
    self.resetPose()
    for i in range(100):
      p.stepSimulation()

  def buildMotorIdList(self):
    self.motorIdList.append(self.jointNameToId['shoulder_pan_joint'])
    self.motorIdList.append(self.jointNameToId['shoulder_lift_joint'])
    self.motorIdList.append(self.jointNameToId['elbow_joint'])
    self.motorIdList.append(self.jointNameToId['wrist_1_joint'])
    self.motorIdList.append(self.jointNameToId['wrist_2_joint'])
    self.motorIdList.append(self.jointNameToId['wrist_3_joint'])
    self.motorIdList.append(self.jointNameToId['left_finger_joint'])
    self.motorIdList.append(self.jointNameToId['right_finger_joint'])


  def reset(self):
    self.id = p.loadURDF("ur_e_description/urdf/ur5e.urdf",basePosition=[-0.6,0,0])
    self.logid = 0
    self.kp = 0.01
    self.kd = 0.01
    self.maxForce = 1000
    self.maxVelocity = 0.05
    self.nMotors = 8
    self.anglesInit = [0,-1.04719,0.851398,-1.420797,-1.570797,-2.3,-0.6,0.86]
    self.anglesLB = [-0.4,-1.4,0,-3.14,-3.14,-3.14,-0.7,0.75]
    self.anglesUB = [0.3,-0.9,1.8,3.14,3.14,3.14,-0.5,0.95]
    self.motorIdList = []
    self.motorDir = [1, 1, 1, 1, 1, 1, 1, 1]
    self.buildJointNameToIdDict()
    self.buildMotorIdList()

  def setMotorAngleById(self, motorId, desiredAngle):
    p.setJointMotorControl2(bodyIndex=self.id, jointIndex=motorId, controlMode=p.POSITION_CONTROL, targetPosition=desiredAngle, positionGain=self.kp, velocityGain=self.kd, force=self.maxForce,maxVelocity = self.maxVelocity)

  def setMotorAngleByName(self, motorName, desiredAngle):
    self.setMotorAngleById(self.jointNameToId[motorName], desiredAngle)

  def resetPose(self):
    self.setMotorAngleByName('shoulder_pan_joint', self.anglesInit[0])
    self.setMotorAngleByName('shoulder_lift_joint', self.anglesInit[1])
    self.setMotorAngleByName('elbow_joint', self.anglesInit[2])
    self.setMotorAngleByName('wrist_1_joint', self.anglesInit[3])
    self.setMotorAngleByName('wrist_2_joint', self.anglesInit[4])
    self.setMotorAngleByName('wrist_3_joint', self.anglesInit[5])
    self.setMotorAngleByName('left_finger_joint', self.motorDir[6])
    self.setMotorAngleByName('right_finger_joint', self.motorDir[7])

  def getBasePosition(self):
    position, orientation = p.getBasePositionAndOrientation(self.id)
    return position

  def getBaseOrientation(self):
    position, orientation = p.getBasePositionAndOrientation(self.id)
    return orientation

  def applyAction(self, motorCommands):
    motorCommandsWithDir = np.multiply(motorCommands, self.motorDir)
    for i in range(self.nMotors):
      self.setMotorAngleById(self.motorIdList[i], motorCommandsWithDir[i])

  def getMotorAngles(self):
    motorAngles = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.id, self.motorIdList[i])
      motorAngles.append(jointState[0])
    motorAngles = np.multiply(motorAngles, self.motorDir)
    return motorAngles

  def getMotorVelocities(self):
    motorVelocities = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.id, self.motorIdList[i])
      motorVelocities.append(jointState[1])
    motorVelocities = np.multiply(motorVelocities, self.motorDir)
    return motorVelocities

  def getMotorTorques(self):
    motorTorques = []
    for i in range(self.nMotors):
      jointState = p.getJointState(self.id, self.motorIdList[i])
      motorTorques.append(jointState[3])
    motorTorques = np.multiply(motorTorques, self.motorDir)
    return motorTorques

  def getAnglesInit(self):
    return self.anglesInit

  def RandomsetPose(self):
    commands = []
    for i in range(self.nMotors):
        commands.append(np.random.uniform(self.anglesLB[i],self.anglesUB[i]))
    self.applyAction(commands)

  def Move2TargetPos(self,Tarpos,Tarorn,EndEffectorIndex):
    Tarorn_Q = p.getQuaternionFromEuler(Tarorn)
    ll=self.anglesLB
    ul=self.anglesUB
    jr=[0.7,0.5,3,6.28,6.28,6.28,0.5,0.5]
    rp=self.anglesInit
    jd=[0.1,0.1,0.1,0.1,0.1,0.1,0.1,0.1]
    jointPoses = p.calculateInverseKinematics(self.id,EndEffectorIndex,Tarpos,Tarorn_Q,ll,ul,jr,rp)
    self.applyAction(jointPoses)

  #def logStart(self):
    #self.logid = p.startStateLogging(loggingType=p.STATE_LOGGING_CONTACT_POINTS,fileName="LOG00000.TXT")

  #def logEnd(self):
    #p.stopStateLogging(self.logid)

  def getData(self,LinkIndex):
    data = p.getLinkState(self.id,LinkIndex)
    return data
