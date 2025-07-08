from etils import epath
import onnxruntime as rt

import os
import select
import numpy as np
from scipy.spatial.transform import Rotation
import time

import lcm
from exlcm import mot_cmd
from exlcm import imu_cmd

_HERE = epath.Path(__file__).parent
_ONNX_DIR = _HERE / "onnx"

class OnnxController:
  """ONNX controller rollout."""

  def __init__(
      self,
      policy_path: str,
      default_angles: np.ndarray,
      n_substeps: int,
      action_scale: float = 0.5,
  ):
    self._output_names = ["continuous_actions"]
    self._policy = rt.InferenceSession(
        policy_path, providers=["CPUExecutionProvider"]
    )

    self._action_scale = action_scale
    self._default_angles = default_angles
    
    # --- Local state buffers ---
    num_joints = len(default_angles)
    self._last_act = np.zeros(num_joints, dtype=np.float32)
    self._last_q_tar = np.zeros(num_joints, dtype=np.float32)
    
    self._q_tar = np.zeros(num_joints, dtype=np.float32)
    
    self._q_rec = np.zeros(num_joints, dtype=np.float32)
    self._dq_rec = np.zeros(num_joints, dtype=np.float32)
    self._tau_rec = np.zeros(num_joints, dtype=np.float32)
    
    self._rpy_euler = np.zeros(3, dtype=np.float32)
    self._gyro = np.zeros(3, dtype=np.float32)
    self._acc = np.zeros(3, dtype=np.float32)
    
    self._counter = 0
    self._n_substeps = n_substeps

    self._command = np.array([0.0, 0.0, 0.0])
    self._act_i_reorder = np.array([0,1,2,3,4,5])
    self._n = 0

    # --- Buffer for history obs ---
    self._HIST_LEN = 2
    self.acc_hist = [np.zeros(3)]*self._HIST_LEN
    self.gyro_hist = [np.zeros(3)]*self._HIST_LEN
    self.grav_hist = [np.zeros(3)]*self._HIST_LEN

    # --- LCM setup ---
    self.lc = lcm.LCM()
    self.imu_sub = self.lc.subscribe("IMU_CMD", self.lcm_imu_handle)
    

# LCM handles
  def lcm_imu_handle(self, channel, data):
    msg = imu_cmd.decode(data)
    # Copy also take 0st element from tuple (numpy)
    self._rpy_euler = np.array(msg.rpy)
    self._gyro = np.array(msg.gyro)
    self._acc = np.array(msg.acc)
    
    #print("Received message on channel \"%s\"" % channel)
    #print("   rpy = %s" % str(msg.rpy))
    #print("   gyro = %s" % str(msg.gyro))
    #print("   acc = %s" % str(msg.acc))
    #print("")
    

  # Get observation from low-level hw
  def get_hw_obs(self) -> np.ndarray:

    # IMU based observations
    gyro = self._gyro
    acc = self._acc
    imu_xmat_obj = Rotation.from_euler('ZYX', self._rpy_euler)
    imu_xmat = imu_xmat_obj.as_matrix()
    grav_vec = imu_xmat.T @ np.array([0, 0, -1])

    # Joint based observations, from low-level mot driver
    q = self._q_rec - self._default_angles
    dq = self._dq_rec

    # Stack history observation with current obs
    # Dim = n*(hist_len + 1), flatten into 1d vec
    acc_stacked = np.concatenate(self.acc_hist + [acc])
    gyro_stacked = np.concatenate(self.gyro_hist + [gyro])
    grav_stacked = np.concatenate(self.grav_hist + [grav_vec])

    # Obs with short history, hist_len+1
    obs = np.hstack([
        acc_stacked,
        gyro_stacked,
        grav_stacked,
        q,
        dq,
        self._last_act,
        np.array([0.0, 0.0, 0.0], dtype=np.float32)
    ])
    
    print("debug data")
    print(gyro)
    print(acc)
    print(grav_vec)

    # Update history buffer
    self.acc_hist = self.acc_hist[1:] + [acc.copy()]
    self.gyro_hist = self.gyro_hist[1:] + [gyro.copy()]
    self.grav_hist = self.grav_hist[1:] + [grav_vec.copy()]

    return obs.astype(np.float32)


  def get_ctr(self) -> None:
    self._counter += 1
      
    obs = self.get_hw_obs()
    onnx_input = {"obs": obs.reshape(1, -1)}
    onnx_pred = self._policy.run(self._output_names, onnx_input)[0][0]

    # Get current target joint angle
    self._q_tar = onnx_pred * self._action_scale + self._last_q_tar
    self._last_q_tar = self._q_tar
    self._last_act = onnx_pred.copy()
    
    print("Target Joint angle - ")
    print(self._q_tar)


  def run(self):
    print("Starting ONNX runner control loop at 50Hz...")
    #motor_cmd_msg = motor_cmd_t()
    loop_duration = 1.0 / 50.0  # 50 Hz

    while True:
      loop_start_time = time.monotonic()

      # --- Set lc.fileno() to non-blocking mode ---
      fd = self.lc.fileno()
      os.set_blocking(fd, False)  # Make the file descriptor non-blocking
      rfds, wfds, efds = select.select([fd], [], [], 0.001)  # Non-blocking with timeout=0
      if rfds:
        self.lc.handle() # Spin lcm receiver handle
      
      print("In control loop")

      self.get_ctr()

      # Publish the motor command via LCM


      # Maintain the 50Hz loop rate
      elapsed_time = time.monotonic() - loop_start_time
      sleep_time = loop_duration - elapsed_time
      if sleep_time > 0:
        time.sleep(sleep_time)
        

if __name__ == "__main__":
    
  DEFAULT_JOINT_ANGLES = np.array([0.6, -0.18, -0.9, -0.6, 0.18, 0.9],
                                  dtype=np.float32)
    
  runner = OnnxController(policy_path=(_ONNX_DIR/"bai_policy_0523_h2.onnx").as_posix(),
                          default_angles=DEFAULT_JOINT_ANGLES,
                          n_substeps=1,
                          action_scale=0.5,
                          )
  try:
    runner.run()
  except KeyboardInterrupt:
    print("\nControl loop stopped by user.")