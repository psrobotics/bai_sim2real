import lcm
import time
import numpy as np
import select
import os

# Before running, make sure you have generated the Python LCM types:
# lcm-gen -p exlcm/mot_cmd.lcm
# lcm-gen -p exlcm/mot_state.lcm
from exlcm import mot_cmd
from exlcm import mot_state

class LcmTester:
    """
    A simple class to test the C++ motor driver via LCM.
    It sends sine-wave position commands and listens for state feedback.
    """

    def __init__(self):
        self.lc = lcm.LCM()
        self.last_state_msg = None
        
        # Subscribe to the MOT_STATE channel from the C++ driver
        self.subscription = self.lc.subscribe("MOT_STATE", self.state_handler)
        print("Subscribed to MOT_STATE channel.")

    def state_handler(self, channel, data):
        """Handles incoming motor state messages."""
        self.last_state_msg = mot_state.decode(data)

    def run_sine_test(self):
        """
        Runs a sine-wave test, publishing commands and printing feedback.
        """
        print("Starting sine-wave test at 500 Hz...")
        
        # --- Sine-wave parameters ---
        phase = 0.0
        freq = 1  # Hz
        dt = 0.002  # Loop period (500 Hz)
        p2p = np.pi # Peak-to-peak amplitude
        num_motors = 6

        # Create a reusable command message object
        cmd_msg = mot_cmd()
        cmd_msg.dq = [0.0] * num_motors
        cmd_msg.tau = [0.0] * num_motors
        cmd_msg.kp_tar = 3.5  # Set a default Kp
        cmd_msg.kd_tar = 0.1  # Set a default Kd
        cmd_msg.is_enable = True

        loop_count = 0
        try:
            while True:
                loop_start_time = time.monotonic()

                # --- Handle incoming LCM messages (non-blocking) ---
                # Use select for a non-blocking handle with a tiny timeout
                rfds, _, _ = select.select([self.lc.fileno()], [], [], 0)
                if rfds:
                    self.lc.handle()

                # --- Calculate new sine-wave targets ---
                phase += 2 * np.pi * freq * dt
                targets = np.zeros(num_motors)
                for i in range(num_motors):
                    targets[i] = np.sin(phase + i * 0.2) * p2p
                
                # --- Populate and publish the command message ---
                cmd_msg.timestamp = int(time.time() * 1e6)
                cmd_msg.q = targets.tolist()
                self.lc.publish("MOT_CMD", cmd_msg.encode())

                # --- Print feedback every 50 cycles ---
                if loop_count % 50 == 0:
                    print(f"--- Loop {loop_count} ---")
                    print(f"Published q_target[0]: {targets[0]:.4f}")
                    if self.last_state_msg:
                        q_measured = self.last_state_msg.q
                        print(f"Received  q_actual[0]: {q_measured[0]:.4f}")
                    else:
                        print("Waiting for first state message from C++ driver...")
                    print("-" * 20)

                # --- Maintain the loop rate ---
                loop_count += 1
                elapsed_time = time.monotonic() - loop_start_time
                sleep_time = dt - elapsed_time
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nSine test stopped by user.")
            # Send a final zero position command
            print("Sending zero position command...")
            cmd_msg.q = [0.0] * num_motors
            cmd_msg.kp_tar = 2.0 # Use a softer Kp for stopping
            cmd_msg.kd_tar = 0.05
            self.lc.publish("MOT_CMD", cmd_msg.encode())
            time.sleep(0.1)


if __name__ == "__main__":
    tester = LcmTester()
    tester.run_sine_test()
