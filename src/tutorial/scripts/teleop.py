#!/usr/bin/env python3
import rospy, sys, select, termios, tty
from fs_msgs.msg import ControlCommand
from std_msgs.msg import Bool

class ControlTest:
    def __init__(self):
        rospy.init_node("control_test", anonymous=True)
        self.control_pub = rospy.Publisher("/fsds/control_command", ControlCommand, queue_size=1)
        self.emergency_pub = rospy.Publisher("/fsds/emergency_stop", Bool, queue_size = 1)

        # 상태 변수
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0

        # Steering 조정 단계 (0.05 radian per keypress)
        self.steer_step = 0.05
        self.steer_limit = 1.0

        self.settings = termios.tcgetattr(sys.stdin)
        rospy.Timer(rospy.Duration(0.1), self.publish_control)  # 10Hz publish rate

        self.run()

    def run(self):
        try:
            print("Use 'W/S' for Throttle/Brake, 'A/D' for Steering, 'SPACE' to reset, 'Q' to quit.")
            while not rospy.is_shutdown():
                key = self.getKey()
                if key == 'w':
                    self.throttle = 1.0
                    self.brake = 0.0
                elif key == 's':
                    self.brake = 1.0
                    self.throttle = 0.0
                elif key == 'a':
                    self.steering = max(self.steering - self.steer_step, -self.steer_limit)
                elif key == 'd':
                    self.steering = min(self.steering + self.steer_step, self.steer_limit)
                elif key == ' ':
                    self.reset_control()
                elif key == 'q':
                    break
                elif key == 'r':
                    self.emergency_publisher()
                elif key == '':
                    # No key pressed, release Throttle/Brake (Steering holds position)
                    self.throttle = 0.0
                    self.brake = 0.0

        except rospy.ROSInterruptException:
            pass
        finally:
            self.reset_terminal()
            self.reset_control()
            self.publish_control(None)
            print("\nExiting...")

    def reset_control(self):
        self.throttle = 0.0
        self.brake = 0.0
        self.steering = 0.0

    def publish_control(self, event):
        msg = ControlCommand()
        msg.throttle = self.throttle
        msg.brake = self.brake
        msg.steering = self.steering
        self.control_pub.publish(msg)
        print(f"Throttle: {msg.throttle:.2f}, Brake: {msg.brake:.2f}, Steering: {msg.steering:.2f}", end='\r')

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)  # Non-blocking with 0.1s timeout
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def reset_terminal(self):
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
    
    def emergency_publisher(self):
        self.emergency = True
        self.emergency_pub.publish(self.emergency)
        


if __name__ == '__main__':
    try:
        ControlTest()
    except rospy.ROSInterruptException:
        pass
