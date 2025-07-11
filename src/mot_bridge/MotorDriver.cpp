#include "motor_driver/MotorDriver.hpp"

namespace motor_driver
{

  MotorDriver::MotorDriver(const std::vector<int> &motor_ids,
                           const char *motor_can_socket,
                           MotorType motor_type = MotorType::AK80_6_V1p1)
      : motor_type_(motor_type),
        motor_ids_(motor_ids),
        motor_CAN_interface_(motor_can_socket)
  {
    // Set Motor Parameters According to Motor Type

    switch (motor_type_)
    {

    case MotorType::AK80_6_V1p1:
      std::cout << "Using Motor Type AK80-6 V1.1" << std::endl;
      current_params_ = default_params::AK80_6_V1p1_params;
      break;

    default:
      perror("Specified Motor Type Not Found!!");
    }

    // Initialize all Motors to not enabled.
    // TODO: Enable enabled check better across multiple objects of this class.
    for (int motor_id : motor_ids_)
      is_motor_enabled_[motor_id] = false;
  }

  // Receive multiple frames with a timeout, nonblocking
  std::map<int, motorState> MotorDriver::receive_frames(int num_frames_to_receive)
  {
    std::map<int, motorState> motor_state_map;
    auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::milliseconds(5); // 5ms timeout

    while (motor_state_map.size() < num_frames_to_receive)
    {
      if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
      {
        motorState state = decodeCANFrame(CAN_reply_msg_);
        motor_state_map[state.motor_id] = state;
      }

      // Check for timeout
      auto current_time = std::chrono::steady_clock::now();
      if (current_time - start_time > timeout)
      {
        //perror("MotorDriver::receive_frames() - Timeout reached. Not all frames received.");
        break;
      }
    }
    return motor_state_map;
  }

  // Helper function for blocking setup/teardown functions
  std::map<int, motorState> MotorDriver::receive_frames_blocking(int num_frames_to_receive)
  {
      std::map<int, motorState> motor_state_map;
      auto start_time = std::chrono::steady_clock::now();
      // Use a very long timeout as a safety measure to prevent infinite loops if a motor is disconnected.
      const auto safety_timeout = std::chrono::microseconds(300);

      while (motor_state_map.size() < num_frames_to_receive)
      {
          if (motor_CAN_interface_.receiveCANFrame(CAN_reply_msg_))
          {
              motorState state = decodeCANFrame(CAN_reply_msg_);
              motor_state_map[state.motor_id] = state;
          }

          // Safety timeout check
          if (std::chrono::steady_clock::now() - start_time > safety_timeout)
          {
              std::cerr << "MotorDriver::receive_frames_blocking() - SAFETY TIMEOUT REACHED. Received "
                        << motor_state_map.size() << "/" << num_frames_to_receive << " frames." << std::endl;
              break;
          }
      }
      return motor_state_map;
  }

  std::map<int, motorState> MotorDriver::enableMotor(
      const std::vector<int> &enable_motor_ids)
  {
      for (int motor_id : enable_motor_ids)
      {
          motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorEnableMsg);
          usleep(motorReplyWaitTime);
      }
      // Use the blocking receive function
      auto states = receive_frames_blocking(enable_motor_ids.size());
      for(auto const& [id, val] : states) {
          is_motor_enabled_[id] = true;
      }
      return states;
  }

  std::map<int, motorState> MotorDriver::disableMotor(
      const std::vector<int> &disable_motor_ids)
  {
      for (int motor_id : disable_motor_ids)
      {
          encodeCANFrame(default_msgs::zeroCmdStruct, CAN_msg_);
          motor_CAN_interface_.sendCANFrame(motor_id, CAN_msg_);
          usleep(motorReplyWaitTime);
      }
      for (int motor_id : disable_motor_ids)
      {
          motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorDisableMsg);
      }
      
      // We expect 2 replies per motor; use the blocking receive function
      auto states = receive_frames_blocking(disable_motor_ids.size() * 2);
      for(auto const& [id, val] : states) {
          is_motor_enabled_[id] = false;
      }
      return states;
  }

  std::map<int, motorState> MotorDriver::setZeroPosition(
      const std::vector<int> &zero_motor_ids)
  {
      for (int motor_id : zero_motor_ids)
      {
          motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorSetZeroPositionMsg);
          usleep(motorReplyWaitTime);
      }
      // Use the blocking receive function
      return receive_frames_blocking(zero_motor_ids.size());
  }
  /*
  std::map<int, motorState> MotorDriver::enableMotor(
      const std::vector<int> &enable_motor_ids)
  {
    for (int motor_id : enable_motor_ids)
    {
      motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorEnableMsg);
    }
    auto states = receive_frames(enable_motor_ids.size());
    for (auto const &[id, val] : states)
    {
      is_motor_enabled_[id] = true;
    }
    return states;
  }

  std::map<int, motorState> MotorDriver::disableMotor(
      const std::vector<int> &disable_motor_ids)
  {
    for (int motor_id : disable_motor_ids)
    {
      encodeCANFrame(default_msgs::zeroCmdStruct, CAN_msg_);
      motor_CAN_interface_.sendCANFrame(motor_id, CAN_msg_);
    }
    for (int motor_id : disable_motor_ids)
    {
      motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorDisableMsg);
    }

    // We expect 2 replies per motor
    auto states = receive_frames(disable_motor_ids.size() * 2);
    for (auto const &[id, val] : states)
    {
      is_motor_enabled_[id] = false;
    }
    return states;
  }

  std::map<int, motorState> MotorDriver::setZeroPosition(
      const std::vector<int> &zero_motor_ids)
  {
    for (int motor_id : zero_motor_ids)
    {
      motor_CAN_interface_.sendCANFrame(motor_id, default_msgs::motorSetZeroPositionMsg);
    }
    return receive_frames(zero_motor_ids.size());
  }
  */

  std::map<int, motorState> MotorDriver::sendRadCommand(
      const std::map<int, motorCommand> &motor_rad_commands)
  {
    for (const auto &command_pair : motor_rad_commands)
    {
      int cmd_motor_id = command_pair.first;
      const motorCommand &cmd_to_send = command_pair.second;
      encodeCANFrame(cmd_to_send, this->CAN_msg_);
      motor_CAN_interface_.sendCANFrame(cmd_motor_id, CAN_msg_);
    }
    return receive_frames(motor_rad_commands.size());
  }

  const motorParams &MotorDriver::getMotorParams() const
  {
    return current_params_;
  }

  void MotorDriver::setMotorParams(const motorParams &new_params)
  {
    current_params_ = new_params;
  }

  motorState MotorDriver::decodeCANFrame(
      const unsigned char *CAN_reply_msg) const
  {
    // unpack ints from can buffer
    int id = CAN_reply_msg[0];

    int p_int = (CAN_reply_msg[1] << 8) | CAN_reply_msg[2];
    int v_int = (CAN_reply_msg[3] << 4) | (CAN_reply_msg[4] >> 4);
    int i_int = ((CAN_reply_msg[4] & 0xF) << 8) | CAN_reply_msg[5];
    // convert unsigned ints to floats
    float p =
        uint_to_float(p_int, current_params_.P_MIN, current_params_.P_MAX, 16);
    float v =
        uint_to_float(v_int, current_params_.V_MIN, current_params_.V_MAX, 12);
    float i = uint_to_float(i_int, -current_params_.T_MAX, current_params_.T_MAX,
                            12); // here -T_MAX, in encode T_MIN

    motorState state{.motor_id = id,
                     .position = p * current_params_.AXIS_DIRECTION,
                     .velocity = v * current_params_.AXIS_DIRECTION,
                     .torque = i * current_params_.AXIS_DIRECTION};

    return state;
  }

  void MotorDriver::encodeCANFrame(const motorCommand &cmd_to_send,
                                   unsigned char *CAN_msg) const
  {
    float p_des = cmd_to_send.p_des * current_params_.AXIS_DIRECTION;
    float v_des = cmd_to_send.v_des * current_params_.AXIS_DIRECTION;
    float tau_ff = cmd_to_send.tau_ff * current_params_.AXIS_DIRECTION;

    // Apply Saturation based on the limits
    p_des = fminf(fmaxf(current_params_.P_MIN, p_des), current_params_.P_MAX);
    v_des = fminf(fmaxf(current_params_.V_MIN, v_des), current_params_.V_MAX);
    tau_ff = fminf(fmaxf(current_params_.T_MIN, tau_ff), current_params_.T_MAX);
    float kp = fminf(fmaxf(current_params_.KP_MIN, cmd_to_send.kp),
                     current_params_.KP_MAX);
    float kd = fminf(fmaxf(current_params_.KD_MIN, cmd_to_send.kd),
                     current_params_.KD_MAX);

    // convert floats to unsigned ints
    int p_int =
        float_to_uint(p_des, current_params_.P_MIN, current_params_.P_MAX, 16);
    int v_int =
        float_to_uint(v_des, current_params_.V_MIN, current_params_.V_MAX, 12);
    int kp_int =
        float_to_uint(kp, current_params_.KP_MIN, current_params_.KP_MAX, 12);
    int kd_int =
        float_to_uint(kd, current_params_.KD_MIN, current_params_.KD_MAX, 12);
    int t_int =
        float_to_uint(tau_ff, current_params_.T_MIN, current_params_.T_MAX, 12);

    // pack ints into the can message
    CAN_msg[0] = p_int >> 8;
    CAN_msg[1] = p_int & 0xFF;
    CAN_msg[2] = v_int >> 4;
    CAN_msg[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    CAN_msg[4] = kp_int & 0xFF;
    CAN_msg[5] = kd_int >> 4;
    CAN_msg[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    CAN_msg[7] = t_int & 0xff;
  }

  int MotorDriver::float_to_uint(float x, float x_min, float x_max,
                                 int bits) const
  {
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
  }

  float MotorDriver::uint_to_float(int x_int, float x_min, float x_max,
                                   int bits) const
  {
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
  }

} // namespace motor_driver