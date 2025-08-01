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

class hw_wrapper:
    '''The robot hardware wrapper class'''
    def __init__(
      self,
      default_angles: np.ndarray,
      action_scale: float = 0.5,
    ):
        # lcm topic names
        self._chn_mot_cmd = "MOT_CMD"
        self._chn_mot_state = "MOT_STATE"
        self._chn_imu = "IMU_STATE"