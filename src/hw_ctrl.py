from etils import epath
import onnxruntime as rt

import os
import select
import numpy as np
from scipy.spatial.transform import Rotation
import time

import lcm
from exlcm import mot_cmd
from exlcm import mot_state
from exlcm import imu_cmd

from dataclasses import dataclass, field

from utils.rt_loop import loop_func, real_time_loop

import mujoco
import mujoco.viewer
import time

_HERE = epath.Path(__file__).parent
_ONNX_DIR = _HERE / "onnx"

@dataclass
class imu_state:
    rpy: np.ndarray = field(default_factory=lambda: np.zeros(3))
    gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acc: np.ndarray = field(default_factory=lambda: np.zeros(3))

@dataclass
class joint_state:
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))
    dq: np.ndarray = field(default_factory=lambda: np.zeros(6))
    tau_est: np.ndarray = field(default_factory=lambda: np.zeros(6))

@dataclass
class joint_act:
    q: np.ndarray = field(default_factory=lambda: np.zeros(6))
    kp: float = 0.0
    kd: float = 0.0
    calibrate: bool = False
    enable: bool = True
     
class hw_wrapper:
    '''The robot hardware wrapper class'''
    def __init__(
      self,
      default_q: np.ndarray,
      policy_path: str,
      ctrl_dt: float = 0.02,
      act_scale: float = 0.5,
      kp: float = 5.0,
      kd: float = 0.05,
    ):
        # lcm topic names
        self._chn_mot_cmd = "MOT_CMD"
        self._chn_mot_state = "MOT_STATE"
        self._chn_imu = "IMU_STATE"
        # lcm init
        self.lc = lcm.LCM()
        self.imu_sub = self.lc.subscribe(self._chn_imu, self.imu_state_handle)
        self.mot_sub = self.lc.subscribe(self._chn_mot_state,self.mot_state_handle)

        self.mot_state_msg = mot_state()
        self.mot_cmd_msg = mot_cmd()
        self.imu_state_msg = imu_state()
        
        # hw states
        self.imu = imu_state()
        self.joint_state = joint_state()
        self.joint_act = joint_act()
        
        # hw params
        self.reduaction_ratio = 9.0
        # Mot dir defiend on low-level
        self.mot_dir = np.array([-1, 1, -1, -1, 1, -1])
        self.joint_offset = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        # Init motor kp
        self.joint_act.kp = 0.7 #/(self.reduaction_ratio*self.reduaction_ratio)
        self.joint_act.kd = 0.08
        
        # Policy onnx params
        self._output_names = ["continuous_actions"]
        self._policy = rt.InferenceSession(
            policy_path, providers=["CPUExecutionProvider"]
        )
        print("ONNX policy init done")

        # Local obs buffer
        self._default_q = default_q
        self._act_scale = act_scale
        
        self._last_action = np.zeros(6, dtype=np.float32)
        self._last_last_action = np.zeros(6, dtype=np.float32)
        self._last_last_last_action = np.zeros(6, dtype=np.float32)
        self._last_gyro = np.zeros(3, dtype=np.float32) 
        self._last_acc = np.zeros(3, dtype=np.float32) 

        self._phase = np.array([0.0, np.pi])
        self._gait_freq = 1.0
        self._phase_dt = 2 * np.pi * self._gait_freq * ctrl_dt
        
        # Global command
        self.command = np.array([0.3, 0.0, 0.0])
        
    
    def imu_state_handle(self, channel, data):
        msg = imu_cmd.decode(data)
        self.imu_state_msg = msg
        #print("Received message on channel \"%s\"" % channel)
        #print("   rpy = %s" % str(msg.rpy))
        #print("   gyro = %s" % str(msg.gyro))
        #print("   acc = %s" % str(msg.acc))
        #print("\n")
        
    def mot_state_handle(self, channel, data):
        msg = mot_state.decode(data)
        self.mot_state_msg = msg
        #print("Received message on channel \"%s\"" % channel)
        #print("   pos = %s" % str(msg.q))
        
    def lcm_loop(self):
        # Non-blocking reading all lcm subs
        rfds, _, _ = select.select([self.lc.fileno()], [], [], 0)
        if rfds:
            self.lc.handle()
    
    # Get hardware observation from sensors
    def get_obs(self):
        gyro = self.imu.gyro
        acc = self.imu.acc
        imu_xmat_obj = Rotation.from_euler('yxz', self.imu.rpy, degrees=False)
        imu_xmat = imu_xmat_obj.as_matrix()
        gravity = imu_xmat.T @ np.array([0, 0, -1])
        
        q = self.joint_state.q - self._default_q
        dq = self.joint_state.dq
        
        phase = np.concatenate([np.cos(self._phase), np.sin(self._phase)])
        
        obs = np.hstack([
            gyro,
            self._last_gyro,
            acc,
            self._last_acc,
            gravity,
            self.command,
            q,
            dq,
            self._last_action,
            self._last_last_action,
            self._last_last_last_action,
            phase,
        ])

        self._last_gyro = gyro
        self._last_acc = acc
        
        return obs.astype(np.float32)
    
        
    def get_ctrl(self):
        # First update states
        self.update_state()
        
        # Get obs
        obs = self.get_obs()
        onnx_input = {"obs": obs.reshape(1, -1)}
        act_pred = self._policy.run(self._output_names, onnx_input)[0][0]

        self._last_last_last_action = self._last_last_action
        self._last_last_action = self._last_action
        self._last_action = act_pred.copy()
        
        # Get target joint
        q_tar = act_pred*self._act_scale + self._default_q
        
        print(q_tar)
      
        # Set initial zero position, debug
        self.joint_act.q = np.zeros(6)
        
        self.joint_act.q = q_tar

        
    def update_state(self):
        # Update state variable from received low-level states
        # Copy over imu states, history
        self.imu.rpy = np.array(self.imu_state_msg.rpy) / 180.0* np.pi
        self.imu.gyro = np.array(self.imu_state_msg.gyro)
        self.imu.acc = np.array(self.imu_state_msg.acc) 
        # Low_mot_state to mujoco state
        _id_c = [2,1,0,3,4,5]
        mot_q = np.array(self.mot_state_msg.q)
        mot_dq = np.array(self.mot_state_msg.dq)
        mot_tau = np.array(self.mot_state_msg.tau)

        self.joint_state.q = ((mot_q * self.mot_dir) / self.reduaction_ratio + self.joint_offset)[_id_c]
        self.joint_state.dq = ((mot_dq * self.mot_dir) / self.reduaction_ratio)[_id_c]
        self.joint_state.tau_est = (mot_tau * self.reduaction_ratio * self.mot_dir)[_id_c]
        
    def send_mot_cmd(self):
        # Mujoco state to Low_mot cmd, running at 500hz
        _id_i = [2,1,0,3,4,5]
        q_cmd = (self.joint_act.q - self.joint_offset) * self.reduaction_ratio 
        self.mot_cmd_msg.q = q_cmd[_id_i] * self.mot_dir
        self.mot_cmd_msg.dq = np.zeros(6)
        self.mot_cmd_msg.tau = np.zeros(6)
        self.mot_cmd_msg.kp_tar = self.joint_act.kp
        self.mot_cmd_msg.kd_tar = self.joint_act.kd
        self.mot_cmd_msg.is_enable = self.joint_act.enable
        # TODO: Joint limit safety check
        
        # Send to lcm
        self.lc.publish(self._chn_mot_cmd, self.mot_cmd_msg.encode())
        
    
if __name__ == "__main__":
    
    # Init the hw wrapper
    default_q = np.array([0.6, 0.12, 0.9, -0.6, -0.12, -0.9])
    onnx_path = "onnx/bai_v2_policy.onnx"
    dt = 0.02
    act_scale = 0.5
    kp = 5.0
    kd = 0.1
    bpd_bai = hw_wrapper(default_q, onnx_path, dt, act_scale, kp, kd)
    
    # Add periodic tasks
    # lcm receive, running 250hz
    lcm_recv_loop = loop_func(name="lcm_recv", period=0.002, cb=bpd_bai.lcm_loop)
    # Motor cmd loop, running 500hz
    mot_cmd_loop = loop_func(name="mot_cmd", period=0.002, cb=bpd_bai.send_mot_cmd)
    # Main control loop, 50hz
    main_ctrl_loop = loop_func(name="main_ctrl", period=0.02, cb=bpd_bai.get_ctrl)
    

    try:
        main_ctrl_loop.start()
        lcm_recv_loop.start()
        mot_cmd_loop.start()
        print("Running all control loops. Press Ctrl+C to exit early.")
        
        # Init mujoco visualizer for debug
        mjcf_path = "xmls/bai_v2.xml"
        model = mujoco.MjModel.from_xml_path(mjcf_path)
        data = mujoco.MjData(model)
        
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                # Get the latest joint state from the hardware wrapper
                hw_q = bpd_bai.joint_state.q
                hw_rpy = bpd_bai.imu.rpy
                
                rot: Rotation = Rotation.from_euler('yxz', hw_rpy, degrees=False)
                qx, qy, qz, qw = rot.as_quat()
                quat_wxyz = (qw, qx, qy, qz)
                
                data.qpos[0:3] = np.array([0.0, 0.0, 0.0])
                data.qpos[3:7] = quat_wxyz #np.array([1.0, 0.0, 0.0, 0.0])
                data.qpos[7:] = hw_q
                mujoco.mj_step(model, data)
                viewer.sync()
                
                # Sleep to not overwhelm the CPU
                time.sleep(0.02) 
                
    except KeyboardInterrupt:
        print("\nCtrl+C received.")
    finally:
        mot_cmd_loop.shutdown()
        lcm_recv_loop.shutdown()
        main_ctrl_loop.shutdown()
        
    print("\nExample script finished.")
