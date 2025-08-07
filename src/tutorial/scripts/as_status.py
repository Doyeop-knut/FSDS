#!/usr/bin/env python3
import rospy, sys, select, termios, tty
from fs_msgs.msg import GoSignal, FinishedSignal
from std_msgs.msg import String,Bool

class AS_Status:
    def __init__(self):
        rospy.init_node("as_status", anonymous=True)
        self.control_pub = rospy.Publisher("/fsds/AS_status", String, queue_size=1)

        self.go_sub = rospy.Subscriber("/fsds/signal/go",GoSignal,self.go_cb)
        self.emergency = rospy.Subscriber("/fsds/emergency_stop",Bool,self.emergency_cb)
        self.finish_pub = rospy.Publisher("/fsds/signal/finished",FinishedSignal, queue_size =1)
        self.status = 0
        self.is_go = False
        self.is_emergency = False

        self.settings = termios.tcgetattr(sys.stdin)
        rospy.Timer(rospy.Duration(0.1), self.publish_control)  # 10Hz publish rate

        self.run()

    def run(self):
        try:
            print("Use 'W/S' for update status, 'SPACE' to reset, 'Q' to quit.")
            while not rospy.is_shutdown():
                key = self.getKey()
                if key == 'w':
                    self.status += 1
                elif key == "s":
                    self.status -= 1
                elif key == ' ':
                    self.reset_control()
                elif key == 'q':
                    break
                
                if self.status < 0:
                    self.status = 0
                if self.status > 4:
                    self.status = 4
                print("status = ", self.status)

                if self.is_go == True and self.status == 1:
                    self.status = 2
                
                if self.is_emergency == True:
                    self.status = 3


        except rospy.ROSInterruptException:
            pass
        finally:
            self.reset_terminal()
            self.reset_control()
            self.publish_control(None)
            print("\nExiting...")

    def reset_control(self):
        self.status = 0

    def go_cb(self,data):
        self.mission = data.mission
        self.track = data.track

        self.is_go = True
    def emergency_cb(self,data):
        self.is_emergency = data.data

        print(self.is_emergency)

    def publish_control(self,event):
        msg = String()
        if self.status == 0:
            print("AS OFF")
            msg.data = "AS OFF"
        elif self.status == 1:
            print("AS READY")
            msg.data = "AS READY"
        elif self.status == 2:
            print("AS Driving")
            msg.data = "AS Driving"
        elif self.status == 3:
            print("AS Emergency")
            msg.data = "AS Emergency"
        elif self.status == 4:
            print("AS Finished")
            msg.data = "AS Finished"
            self.publish_finish()
        
        
        self.control_pub.publish(msg)

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

    def publish_finish(self):
        finish = FinishedSignal()
        finish.placeholder = True
        self.finish_pub.publish(finish)

if __name__ == '__main__':
    try:
        AS_Status()
    except rospy.ROSInterruptException:
        pass
