#!/usr/bin/env python3
import rospy, random, time, yaml, subprocess, signal
from std_msgs.msg import Float32, Int32, String, UInt32 # Added UInt32
from std_srvs.srv import Empty
from fs_msgs.msg import GoSignal, ExtraInfo # Added ExtraInfo
from fs_msgs.srv import Reset

class Metrics:
    def __init__(self):
        self.reset()
        self._first_doo_counter_received = False # Flag to indicate if first doo_counter is received
        self.initial_cone_hits = 0
        rospy.Subscriber("/fsds/testing_only/extra_info", ExtraInfo, self._cb_extra_info) # Subscribing to ExtraInfo
        rospy.Subscriber("/fsds/AS_status", String, self._cb_state)
    def reset(self):
        self.lap_time = []
        self.cone_hits = 0
        self.state = "AS_OFF"
        self.start_t = None
        self._first_doo_counter_received = False # Reset flag
        self.initial_cone_hits = 0 # Reset initial cone hits
    def _cb_extra_info(self, msg: ExtraInfo): # New callback for ExtraInfo
        # Handle lap_time from ExtraInfo
        # Assuming msg.lap_time is a list of lap times, take the last one
        if isinstance(msg.laps, list) and len(msg.laps) > 0:
            self.lap_time = msg.laps[-1]
        elif isinstance(msg.laps, (float, int)): # If it's a single float/int, use it directly
            self.lap_time = float(msg.laps)
        else:
            rospy.logwarn(f"Unexpected lap time data type in ExtraInfo: {type(msg.laps)}, data: {msg.laps}")
            self.lap_time = None
        print(f"Lap time: {self.lap_time}")

        # Handle doo_counter from ExtraInfo
        self.cone_hits = msg.doo_counter
        if not self._first_doo_counter_received:
            self.initial_cone_hits = self.cone_hits
            self._first_doo_counter_received = True
        print(f"Cone hits: {self.cone_hits}")
    def _cb_state(self, msg): self.state = msg.data

def set_params(pairs):
    for key, val in pairs.items():
        rospy.set_param(key, val)

def score_fn(m: Metrics):
    # Score function: lap time + penalty for cone hits
    if m.lap_time is None: return 1e9 # Return a very high score if the lap was not completed
    try:
        lap_time_float = float(m.lap_time)
    except (ValueError, TypeError):
        rospy.logerr(f"Invalid lap time received: {m.lap_time}. Returning high score.")
        return 1e9 # Return a very high score if lap_time is not a valid number

    # Calculate cone hits for this episode only
    episode_cone_hits = max(0, m.cone_hits - m.initial_cone_hits) # Ensure it's not negative

    return (1.0*lap_time_float + 200.0*episode_cone_hits)

def start_episode():
    # Restart the node to ensure it loads any new parameters from the server
    # and starts in a clean state, as if the launch file was restarted.
    rospy.loginfo("Restarting /formula_autonomous_system node...")
    try:
        # The correct node name is /formula_autonomous_system.
        subprocess.call(["rosnode", "kill", "/formula_autonomous_system"])
        time.sleep(3.0)  # Give time for the node to restart
    except Exception as e:
        rospy.logerr(f"Failed to kill node: {e}")

    # 1) Reset simulator/map if reset service is available
    try:
        rospy.wait_for_service("/fsds/reset", timeout=2.0)
        rospy.ServiceProxy("/fsds/reset", Reset)()
    except Exception:
        pass
    # 2) Send GO signal using the correct message type
    # pub = rospy.Publisher("/fsds/signal/go", GoSignal, queue_size=1, latch=True)
    # time.sleep(0.5) # Give time for publisher to establish connection
    # go_msg = GoSignal()
    # go_msg.mission = "trackdrive" # Set a valid mission
    # pub.publish(go_msg)
    # rospy.loginfo("GO signal sent.")
    rospy.loginfo("Episode started.")

def wait_episode(m: Metrics, timeout=100.0):
    t0 = time.time()
    start_cone_hit = m.cone_hits
    while not rospy.is_shutdown():
        if m.lap_time is not None:        # Lap completion event
            return True
        if (time.time()-t0) > timeout:    # Timeout
            rospy.logwarn("Episode timed out.")
            return False
        if m.cone_hits - start_cone_hit>= 1:              # Stop immediately on cone hit
            rospy.logwarn("Cone hit detected, calling reset and restarting node.")
            try:
                # 1. Call the reset service using rospy.ServiceProxy for reliability
                rospy.loginfo("Calling reset service...")
                rospy.wait_for_service("/fsds/reset", timeout=2.0)
                reset_service = rospy.ServiceProxy("/fsds/reset", Reset)
                reset_service(waitOnLastTask=False)
                
                # 2. Restart the node
                rospy.loginfo("Restarting /formula_autonomous_system node after cone hit...")
                # The correct node name is /formula_autonomous_system.
                subprocess.call(["rosnode", "kill", "/formula_autonomous_system"])
                time.sleep(3.0) # Give time for the node to restart
            except Exception as e:
                rospy.logerr(f"Failed to call reset service or kill node: {e}")
            return False
        time.sleep(0.05)

def update_config_yaml(params, config_path):
    """Reads, updates, and writes the config.yaml file with new parameters."""
    try:
        with open(config_path, 'r') as f:
            config_data = yaml.safe_load(f)
            if config_data is None: # Handle empty or null YAML file
                pass
    except (IOError, yaml.YAMLError) as e:
        rospy.logerr(f"Error reading {config_path}: {e}")
        return

    # Create a nested structure from the flat parameter list
    nested_params = {}
    for key, val in params.items():
        parts = key.strip('/').split('/')
        d = nested_params
        for part in parts[:-1]:
            d = d.setdefault(part, {})
        d[parts[-1]] = val

    # Deep merge the new params into the config data
    def deep_update(d, u):
        for k, v in u.items():
            if isinstance(v, dict) and k in d and isinstance(d[k], dict):
                d[k] = deep_update(d[k], v)
            else:
                d[k] = v
        return d

    updated_config = deep_update(config_data, nested_params)

    try:
        with open(config_path, 'w') as f:
            yaml.safe_dump(updated_config, f, default_flow_style=False)
        rospy.loginfo(f"Updated {config_path} with new tuning parameters.")
    except (IOError, yaml.YAMLError) as e:
        rospy.logerr(f"Error writing to {config_path}: {e}")

def main():
    rospy.init_node("mpc_auto_tuner")
    m = Metrics()
    print(m.lap_time)
    trials = rospy.get_param("~trials", 50) # Increased trials for better search
    config_path = "/home/user/FSDS/src/formula_autonomous_system/config/config.yaml"

    # Search space for MPC mapping_mode parameters from config.yaml
    def sample():
        return {
          "/control/MPC/mapping_mode/w_ey":     random.uniform(1.0, 20.0),
          "/control/MPC/mapping_mode/w_epsi":   random.uniform(1.0, 15.0),
          "/control/MPC/mapping_mode/w_vy":     random.uniform(1.0, 10.0),
          "/control/MPC/mapping_mode/w_r":      random.uniform(1.0, 10.0),
          "/control/MPC/mapping_mode/w_v":      random.uniform(5.0, 25.0),
          "/control/MPC/mapping_mode/w_ax":     random.uniform(0.5, 5.0),
          "/control/MPC/mapping_mode/w_delta":  random.uniform(1.0, 30.0),
          "/control/MPC/mapping_mode/w_dax":    random.uniform(1.0, 15.0),
          "/control/MPC/mapping_mode/w_ddelta": random.uniform(5.0, 150.0),
          # Also tune some vehicle model parameters
        }

    best = (1e12, {})
    for i in range(trials):
        rospy.loginfo(f"[TUNE] Trial {i+1}/{trials}")
        params = sample()
        update_config_yaml(params, config_path)
        set_params(params)

        m.reset()
        start_episode()
        ok = wait_episode(m, timeout=100.0)
        s = score_fn(m) if ok else 1e12
        rospy.loginfo(f"[TUNE] Trial {i+1}: score={s:.2f}, hits={m.cone_hits}, lap_time={m.lap_time}")

        if s < best[0]:
            best = (s, params)
            # Save the best parameters found so far to a temporary file
            save_path = "/tmp/best_mpc_tune.yaml"
            with open(save_path,"w") as f: 
                # Create a nested structure for better readability
                nested_params = {}
                for key, val in best[1].items():
                    parts = key.strip('/').split('/')
                    d = nested_params
                    for part in parts[:-1]:
                        d = d.setdefault(part, {})
                    d[parts[-1]] = val
                yaml.safe_dump(nested_params, f)
            rospy.loginfo(f"[TUNE] ★ Best updated: score={best[0]:.2f} (saved to {save_path})")

    rospy.loginfo(f"[TUNE] DONE. Best score={best[0]:.2f}")
    rospy.loginfo(f"[TUNE] Best params found:\n{yaml.safe_dump(best[1])}")

if __name__ == "__main__":
    main()