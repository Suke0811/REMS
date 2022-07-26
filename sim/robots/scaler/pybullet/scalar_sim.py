#!/usr/bin/env python

import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import time
from scipy.spatial.transform import Rotation as R

import os
import sys


def postion_quat_to_matrix(inpt):
    #input -> [postion, quation]
    #return matrix form
    rot = R.from_quat(inpt[1])
    res = np.eye(4, dtype=np.float32)
    res[0:3,0:3] = rot.as_matrix()
    res[0:3,3] = np.array(inpt[0])
    return res


class pyb_sim(object):
    N_leg = 4
    connectedLinkNameList = [['C_Link00', 'C_Link01'], ['C_Link10', 'C_Link11'], ['C_Link20', 'C_Link21'], ['C_Link30', 'C_Link31']]

    passiveJointNameList = ['B_Joint0', 'E_Joint0', 'B_Joint1', 'E_Joint1', 'B_Joint2', 'E_Joint2', 'B_Joint3', 'E_Joint3']
    passiveJointNum = 8

    bodyJointNameList = ['BodyTopleft_Joint','BodyRight_Joint', 'BodyBot_Joint']
    bodyJointNum = 3

    maxForce = 300
    passiveJointFriction = 0

    def __init__(self, urdf_filename, bodyFixed=True, DoFnum=3, RobotStartPos=[0, 0, 0.5],
                 RobotStartOrientation=[0, 0, 0], delta_t=None):

        self.bodyFixed = bodyFixed
        self.urdf_filename = urdf_filename
        self.RobotStartPos = RobotStartPos
        self.RobotStartOrientation = p.getQuaternionFromEuler(RobotStartOrientation)
        self.delta_t = delta_t
        self.worldTbody = np.eye(4, dtype=np.float32)
        self.worldTbody[0:3, 3] = np.array(self.RobotStartPos)
        self.worldTbody[0:3, 0:3] = np.array(p.getMatrixFromQuaternion(self.RobotStartOrientation)).reshape((3, 3))
        self.bodyTworld = np.linalg.inv(self.worldTbody)

        self.DoFnum = DoFnum
        if DoFnum == 3:
            self.motorJointNameList = ['Shoulder_Joint0', 'A_Joint0', 'F_Joint0', 'Shoulder_Joint1', 'A_Joint1', 'F_Joint1', 'Shoulder_Joint2', 'A_Joint2', 'F_Joint2', 'Shoulder_Joint3', 'A_Joint3', 'F_Joint3']
            self.motorJointNum = 12
            self.HOME_POSE = [0.0, 0,0,np.pi/2, 0,0,np.pi/2, 0,0,np.pi/2, 0,0,np.pi/2]  #The first one is the body motor

        elif DoFnum == 6:
            self.motorJointNameList = ['Shoulder_Joint0', 'A_Joint0', 'F_Joint0','wrist1_Joint0', 'wrist2_Joint0', 'wrist3_Joint0', 
                                       'Shoulder_Joint1', 'A_Joint1', 'F_Joint1','wrist1_Joint1', 'wrist2_Joint1', 'wrist3_Joint1', 
                                       'Shoulder_Joint2', 'A_Joint2', 'F_Joint2','wrist1_Joint2', 'wrist2_Joint2', 'wrist3_Joint2',
                                       'Shoulder_Joint3', 'A_Joint3', 'F_Joint3','wrist1_Joint3', 'wrist2_Joint3', 'wrist3_Joint3']
            self.motorJointNum = 24
            self.HOME_POSE = [0.0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0, 0, 0, np.pi / 2, 0, 0, 0]  #The first one is the body motor
        else:
            print("Invalid DoF number!! Should be 3 or 6.")
            sys.exit()

        self.physicsClient = p.connect(p.GUI)
        self.reset()

    def dance(self):
        count = 0
        direction = 1
        freq = 100
        action_circle_time = 2
        action_range = np.pi/8
        increment = action_range/(action_circle_time*freq/2)
        pos = 0
        self.setTimestep(1.0 / freq)
        while 1:

            if count == int(action_circle_time * freq):
                direction = -1
            if count == 0:
                direction = 1
            for i in range(self.motorJointNum):
                self.HOME_POSE[i] = self.HOME_POSE[i] + direction * increment
            count = count + direction
            self.movetoPose(self.HOME_POSE)

            time.sleep(1.0 / freq)
            self.step()


    def buildJointNameToIdDict(self):
        nJoints = p.getNumJoints(self.RobotId)
        self.jointNameToId = {}
        self.linkNameToId = {}
        for i in range(nJoints):
            jointInfo = p.getJointInfo(self.RobotId, i)
            self.jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
            self.linkNameToId[jointInfo[12].decode('UTF-8')] = jointInfo[0]

    def buildBodyMotorIdList(self):
        self.BodyMotorIdList = []
        for motorJointName in self.bodyJointNameList:
            self.BodyMotorIdList.append(self.jointNameToId[motorJointName])

    def buildMotorIdList(self):
        self.MotorIdList = []
        for motorJointName in self.motorJointNameList:
            self.MotorIdList.append(self.jointNameToId[motorJointName])

    def connectLinks(self):
        for i in range(self.N_leg):
            cid = p.createConstraint(self.RobotId, self.linkNameToId[self.connectedLinkNameList[i][0]],
                                     self.RobotId, self.linkNameToId[self.connectedLinkNameList[i][1]],
                                     p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 0])
            p.changeConstraint(cid, maxForce=self.maxForce * 1000)

    def setPassiveJoints(self):
        for i in range(self.passiveJointNum):
            p.setJointMotorControl2(bodyUniqueId=self.RobotId,
                                    jointIndex=self.jointNameToId[self.passiveJointNameList[i]],
                                    controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=self.passiveJointFriction)


    def resettoPose(self, targetPose):
        newtargetPose = np.array(targetPose)
        for i in range(self.N_leg):
            newtargetPose[i * self.DoFnum + 2] = newtargetPose[i * self.DoFnum + 2] - np.pi / 2
        for i in range(self.motorJointNum):
            p.resetJointState(self.RobotId, self.jointNameToId[self.motorJointNameList[i]], newtargetPose[i])

    def movetoPose(self, targetPose):
        # This function use position control to move the robot to the target pose
        # Input: targetPose -> bodymotor + [shoulder angle, q11, q21, wrist1, wrist2, wrist3]*4 legs, totatally 25 values if 6 DOf
        #                      bodymotor + [shoulder angle, q11, q21]*4 legs, totatally 13 values if 6 DOf
        newtargetPose = np.array(targetPose).reshape(-1)
        bodyAngle = newtargetPose[0]
        newtargetPose = newtargetPose[1:]


        bodyPose = [bodyAngle, -bodyAngle, bodyAngle]
        p.setJointMotorControlArray(self.RobotId, self.BodyMotorIdList, p.POSITION_CONTROL, bodyPose,
                                    forces=[self.maxForce] * self.bodyJointNum)


        for i in range(self.N_leg):
            newtargetPose[i * self.DoFnum + 2] = newtargetPose[i * self.DoFnum + 2] - np.pi / 2
        p.setJointMotorControlArray(self.RobotId, self.MotorIdList, p.POSITION_CONTROL, newtargetPose,
                                    forces=[self.maxForce] * self.motorJointNum)

    def reset(self):

        # Start pybullet and load the plane
        p.resetSimulation()
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.planeId = p.loadURDF("plane.urdf")

        # load the robot from urdf
        self.RobotId = p.loadURDF(self.urdf_filename, self.RobotStartPos, self.RobotStartOrientation,
                                  useFixedBase=self.bodyFixed)

        self.buildJointNameToIdDict()
        self.buildBodyMotorIdList()
        self.buildMotorIdList()

        self.connectLinks()
        self.setPassiveJoints()
        self.movetoPose(self.HOME_POSE)

        if self.delta_t is None:
            p.setRealTimeSimulation(True)
        else:
            self.setTimestep(self.delta_t)

    def getBodyState(self):
        #return the T matrix of body frame in world frame
        bodyDynamicsInfo = p.getDynamicsInfo(self.RobotId,-1)
        urdf_T_CoM = postion_quat_to_matrix([bodyDynamicsInfo[3], bodyDynamicsInfo[4]])
        
        

        bodyCoMinfo = p.getBasePositionAndOrientation(self.RobotId)
        world_T_CoM = postion_quat_to_matrix(bodyCoMinfo)

        CoM_T_urdf = np.linalg.inv(urdf_T_CoM)
        return np.dot(world_T_CoM, CoM_T_urdf)
        


    def fk_with_name(self, name):
        #return the T matrix of any link frame in body frame
        #For 3 DOF scaler, the end effector name is 'Toe_Link'+index number, eg. leg 0 end effector is 'Toe_Link0', the end effector is defined at the tip of the ball
        #For 6 DOF scaler, the end effector name is 'wrist3_Joint'+index number, eg. leg 0 end effector is 'wrist3_Joint0', the end effector is defined at the intersection of the wrist motors' frames
        return self.fk_with_index(self.linkNameToId[name])

    def fk_with_index(self, ind):
        trans = p.getLinkState(self.RobotId, ind)[4]
        orien = p.getLinkState(self.RobotId, ind)[5]
        world_T_Link = postion_quat_to_matrix([trans, orien])
        world_T_body = self.getBodyState()
        body_T_world = np.linalg.inv(world_T_body)

        return np.dot(body_T_world, world_T_Link)

    def step(self):
        p.stepSimulation()

    def setTimestep(self, delta_t):
        p.setTimeStep(delta_t)

    def getJointAngles(self):
        JointAngleList = []
        for i in range(self.motorJointNum):
            JointAngleList.append(p.getJointState(self.RobotId, self.jointNameToId[self.motorJointNameList[i]])[0])
        for i in range(self.N_leg):
            JointAngleList[i * self.DoFnum + 2] = JointAngleList[i * self.DoFnum + 2] + np.pi / 2
        JointAngleList.insert(0, p.getJointState(self.RobotId, self.jointNameToId[self.bodyJointNameList[0]])[0])
        return JointAngleList

    def startRecordingVideo(self, video_path_name):
        # video_path_name: the path and the name of the generated video in mp4 format, for example: videos/round1.mp4
        self.recordId = p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, video_path_name)

    def stopRecordingVideo(self):
        p.stopStateLogging(self.recordId)


