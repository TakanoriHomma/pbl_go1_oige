import pybullet as p
import pybullet_data
import time
import numpy as np

class pybulletenv():

    def __init__(self):
        physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8)
        p.setTimeStep(1./500.)
        planeId = p.loadURDF("plane.urdf")
        urdfFlags = p.URDF_USE_SELF_COLLISION
        self.quadruped = p.loadURDF("a1/urdf/a1.urdf",[0,0,0.4],[0,0,0,1], flags = urdfFlags,useFixedBase=False)
        # self.quadruped = p.loadURDF("a1_dae/urdf/a1.urdf",[0,0,0.4],[0,0,0,1], flags = urdfFlags,useFixedBase=False)
        # self.ground = p.loadURDF('a1/urdf/ground.urdf',[2.5,0,0],[0,0,1,1],flags=urdfFlags,useFixedBase=True)
        self.ground2 = p.loadURDF('a1/urdf/ground2.urdf',[1,0,0],[0,0,1,1],flags=urdfFlags,useFixedBase=True)
        self.ground3 = p.loadURDF('a1/urdf/ground3.urdf',[2.2,0,0],[0,0,1,1],flags=urdfFlags,useFixedBase=True)
        self.init_state = [[[0,0,0.365],[0,0,0,1]],\
        [0.03727735,0.66175963,-1.20017636,-0.028954,0.618814,-1.183148,0.048225,0.690008,-1.254787,-0.050525,0.661355,-1.243304]]
        self.num_joints = p.getNumJoints(self.quadruped)
        self.fixed_joints = 0
        self.joints_info = []
        self.jointIds = []
        self.joints_type = []
        for j in range(self.num_joints):
            self.joints_info.append(p.getJointInfo(self.quadruped,j))
            self.joints_type.append(self.joints_info[j][2])
            if self.joints_type[j]==p.JOINT_PRISMATIC or self.joints_type[j]==p.JOINT_REVOLUTE:
                self.jointIds.append(self.joints_info[j][0])
            else:
                self.fixed_joints += 1
        self.movale_joints = self.num_joints - self.fixed_joints
        self.maxForceId = p.addUserDebugParameter("maxForce",0,100,20)
        self.pred_pos = [None,None]
        self.comp = 0
        p.setRealTimeSimulation(0)

    def do_simulation(self,action,obs,frame_skip=1):
        maxForce = p.readUserDebugParameter(self.maxForceId)
        input_pos = action+obs[-12:]
        input_pos = self.obs_fit_in_limit(input_pos)
        for _ in range(frame_skip):
            for j in range(self.movale_joints):
                p.setJointMotorControl2(self.quadruped, self.jointIds[j], p.POSITION_CONTROL, targetPosition=input_pos[j],force=maxForce)
                p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=40, cameraPitch=-20, cameraTargetPosition=p.getBasePositionAndOrientation(self.quadruped)[0])
            p.stepSimulation() 
    
    def obs_fit_in_limit(self,obs):
        # jointIds = [2, 3, 4, 6, 7, 8, 10, 11, 12, 14, 15, 16]
        # ['FR_hip_angle','FR_thigh_angle','FR_calf_angle','FL_hip_angle','FL_thigh_angle','FL_calf_angle',
        # 'RR_hip_angle','RR_thigh_angle','RR_calf_angle','RL_hip_angle','RL_thigh_angle','RL_calf_angle']
        limit_high = np.array([np.pi/18,np.pi/18*5,-1.0   , np.pi/9,np.pi/18*5,-1.0     , np.pi/18,np.pi/18*5,-1.0   , np.pi/9,np.pi/18*5,-1.0])
        limit_low = -np.array([np.pi/9,np.pi/9,np.pi/3*2 , np.pi/18,np.pi/9,np.pi/3*2 , np.pi/9,np.pi/9,np.pi/3*2 , np.pi/18,np.pi/9,np.pi/3*2])
        for j in range(self.movale_joints):
            if obs[j] < limit_low[j]:
                obs[j] = limit_low[j]
            elif obs[j] > limit_high[j]:
                obs[j] = limit_high[j]
        return obs


    def _get_obs(self):
        curpos = np.array(p.getBasePositionAndOrientation(self.quadruped))
        done = curpos[0][2] < 0.20  or curpos[0][0] > 3#倒れたらやり直し
        if curpos[0][0] > 3:
            self.comp += 1
        curpos[0],curpos[1] = np.array(curpos[0]),np.array(curpos[1])
        curpos[0][:2] = curpos[0][:2] - self.pred_pos
        self.pred_pos = curpos[0][:2]
        curpos = np.concatenate([np.array(curpos[0]),np.array(curpos[1])])
        curpos = np.concatenate([curpos[:3],np.array(p.getEulerFromQuaternion(curpos[3:]))])
        done = abs(curpos[3]) > np.pi/4 or abs(curpos[4]) > np.pi/3 or abs(curpos[5]) > np.pi/2 or done
        curpos = curpos[2:]        
        curvel = np.array(p.getBaseVelocity(self.quadruped))
        curvel = np.concatenate([np.array(curvel[0]),np.array(curvel[1])])
        curpos = np.concatenate([curpos,curvel])
        for j in range(self.movale_joints):
            curpos = np.append(curpos,p.getJointState(self.quadruped, self.jointIds[j])[0])
        # for j in range(self.movale_joints):
        #     curpos = np.append(curpos,p.getJointState(self.quadruped, self.jointIds[j])[1])
        return curpos,done
    
    def reset(self):
        p.resetDebugVisualizerCamera(cameraDistance=2.5, cameraYaw=40, cameraPitch=-20, cameraTargetPosition=p.getBasePositionAndOrientation(self.quadruped)[0])
        random_pos = 0.15*np.random.randn(4)
        random_joint = 0.1*np.random.randn(12)
        height = np.random.rand()*0.01
        # height = np.random.rand()*0.01
        p.resetBasePositionAndOrientation(self.ground2,[1,0,-height],[0,0,1,1])
        # p.resetBasePositionAndOrientation(self.ground3,[1.5,0,-height],[0,0,1,1])
        init_state = self.init_state + np.array([[[0,0,0],random_pos],random_joint])
        p.resetBasePositionAndOrientation(self.quadruped,init_state[0][0],init_state[0][1])
        for j in range(self.movale_joints):
            p.resetJointState(self.quadruped,self.jointIds[j],init_state[1][j])
        init_pos = np.array(p.getBasePositionAndOrientation(self.quadruped))
        init_pos[0],init_pos[1] = np.array(init_pos[0]),np.array(init_pos[1])
        self.pred_pos = init_pos[0][:2]
        init_pos = np.concatenate([np.array(init_pos[0]),np.array(init_pos[1])])
        init_pos = np.concatenate([init_pos[:3],np.array(p.getEulerFromQuaternion(init_pos[3:]))])
        init_pos = init_pos[2:]
        init_vel = np.array(p.getBaseVelocity(self.quadruped))
        init_vel = np.concatenate([np.array(init_vel[0]),np.array(init_vel[1])])
        init_pos = np.concatenate([init_pos,init_vel])
        for j in range(self.movale_joints):
            init_pos = np.append(init_pos,p.getJointState(self.quadruped, self.jointIds[j])[0])
        # for j in range(self.movale_joints):
        #     init_pos = np.append(init_pos,p.getJointState(self.quadruped, self.jointIds[j])[1])
        return init_pos
