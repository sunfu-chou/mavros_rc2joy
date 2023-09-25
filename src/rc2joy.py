#! /usr/bin/env python

import rospy
from mavros_msgs.msg import RCIn
from sensor_msgs.msg import Joy

LOW_RC_LEFT_STICK_X = 1065
HIGH_RC_LEFT_STICK_X = 1933
LOW_RC_LEFT_STICK_Y = 1933
HIGH_RC_LEFT_STICK_Y = 1065

LOW_RC_RIGHT_STICK_X = 1065
HIGH_RC_RIGHT_STICK_X = 1933
LOW_RC_RIGHT_STICK_Y = 1933
HIGH_RC_RIGHT_STICK_Y = 1065

LOW_RC_SW_ENABLE = 1065
HIGH_RC_SW_ENABLE = 1933
LOW_RC_SW_EMERGENCY_STOP = 1065
HIGH_RC_SW_EMERGENCY_STOP = 1933
LOW_RC_SW_MODE = 1057
HIGH_RC_SW_MODE = 1932
LOW_RC_DIFFERENTIAL = 1065
HIGH_RC_DIFFERENTIAL = 1933

RC_SW_TOL = 100

I_RC_LEFT_STICK_X = 3
I_RC_LEFT_STICK_Y = 2
I_RC_RIGHT_STICK_X = 0
I_RC_RIGHT_STICK_Y = 1
I_RC_SW_ENABLE = 7
I_RC_SW_EMERGENCY_STOP = 8
I_RC_SW_MODE = 9
I_RC_DIFFERENTIAL = 6

I_JS_LEFT_STICK_X = 0
I_JS_LEFT_STICK_Y = 1
I_JS_RIGHT_STICK_X = 3
I_JS_RIGHT_STICK_Y = 4

I_JS_SW_EMERGENCY_STOP = 4

I_JS_SW_DIFFERENTIAL = 5

I_JS_MANUAL_MODE = 6
I_JS_AUTO_MODE = 7


def mapping(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def clip_value(value, min_value, max_value):
    """
    Clips a value to be within the specified range [min_value, max_value].
    
    Parameters:
        value (float or int): The value to be clipped.
        min_value (float or int): The minimum value of the range.
        max_value (float or int): The maximum value of the range.
    
    Returns:
        float or int: The clipped value within the range [min_value, max_value].
    """
    return min(max_value, max(min_value, value))

class RC2Joy:
    def __init__(self):
        self.node_name = rospy.get_name()

        self.pub_joy = rospy.Publisher("/joy", Joy, queue_size=1)
        self.sub_rc = rospy.Subscriber("/mavros/rc/in", RCIn, self.rc_callback, queue_size=1)

        self.joy = Joy()
        self.joy.axes = [0.0] * 8
        self.joy.buttons = [0] * 11

        self.last_rcin = RCIn()
        # rospy.Timer(rospy.Duration(1), self.timer_reset_joy)

        self.recieved = False
        self.reset = False

        self.enable = False
        # self.emergencyStop = False
        # self.autoMode = False
        
        self.pub_disable = False
        self.timeout = 0.5

    def rc_callback(self, rc):
        self.recieved = True
        
        # Jot value mapping from RC to Joy
        self.joy.axes[I_JS_LEFT_STICK_X] = mapping(rc.channels[I_RC_RIGHT_STICK_X], LOW_RC_RIGHT_STICK_X, HIGH_RC_RIGHT_STICK_X, 1.0, -1.0)
        self.joy.axes[I_JS_LEFT_STICK_Y] = mapping(rc.channels[I_RC_RIGHT_STICK_Y], LOW_RC_RIGHT_STICK_Y, HIGH_RC_RIGHT_STICK_Y, -1.0, 1.0)
        self.joy.axes[I_JS_RIGHT_STICK_X] = mapping(rc.channels[I_RC_LEFT_STICK_X], LOW_RC_LEFT_STICK_X, HIGH_RC_LEFT_STICK_X, 1.0, -1.0)
        self.joy.axes[I_JS_RIGHT_STICK_Y] = mapping(rc.channels[I_RC_LEFT_STICK_Y], LOW_RC_LEFT_STICK_Y, HIGH_RC_LEFT_STICK_Y, -1.0, 1.0)

        # Add dead zone and clip value
        for i in (I_JS_LEFT_STICK_X, I_JS_LEFT_STICK_Y, I_JS_RIGHT_STICK_X, I_JS_RIGHT_STICK_Y):
            if abs(self.joy.axes[i]) < 0.01:
                self.joy.axes[i] = 0.0
            self.joy.axes[i] = clip_value(self.joy.axes[i], -1.0, 1.0)
        
        # Enable
        self.enable = rc.channels[I_RC_SW_ENABLE] > HIGH_RC_SW_ENABLE - RC_SW_TOL

        # Emergency stop
        emergency_stop = rc.channels[I_RC_SW_EMERGENCY_STOP] > HIGH_RC_SW_EMERGENCY_STOP - RC_SW_TOL
        self.joy.buttons[I_JS_SW_EMERGENCY_STOP] = int(emergency_stop)

        # Differential
        differential = rc.channels[I_RC_DIFFERENTIAL] > HIGH_RC_DIFFERENTIAL - RC_SW_TOL
        self.joy.buttons[I_JS_SW_DIFFERENTIAL] = int(differential)
        
        # Mode
        if len(self.last_rcin.channels) > 0:
            auto_mode = rc.channels[I_RC_SW_MODE] > HIGH_RC_SW_MODE - RC_SW_TOL
            manual_mode = rc.channels[I_RC_SW_MODE] < LOW_RC_SW_MODE + RC_SW_TOL
            last_auto_mode = self.last_rcin.channels[I_RC_SW_MODE] > HIGH_RC_SW_MODE - RC_SW_TOL
            last_manual_mode = self.last_rcin.channels[I_RC_SW_MODE] < LOW_RC_SW_MODE + RC_SW_TOL
            # Auto mode
            if last_manual_mode and auto_mode:
                rospy.loginfo("Auto mode")
                self.joy.buttons[I_JS_AUTO_MODE] = 1
                self.joy.buttons[I_JS_MANUAL_MODE] = 0
            # Manual mode
            elif last_auto_mode and manual_mode:
                rospy.loginfo("Manual mode")
                self.joy.buttons[I_JS_AUTO_MODE] = 0
                self.joy.buttons[I_JS_MANUAL_MODE] = 1
            else:
                self.joy.buttons[I_JS_AUTO_MODE] = 0
                self.joy.buttons[I_JS_MANUAL_MODE] = 0
            
        self.publish()
        self.last_rcin = rc

    def publish(self, force=False):
        # TODO: Handle timeout
        
        self.joy.header.stamp = rospy.Time.now()
        if self.enable or force:
            self.pub_joy.publish(self.joy)
            self.pub_disable = False
        elif not self.pub_disable:
            joy = Joy()
            joy.header.stamp = rospy.Time.now()
            joy.header.frame_id = "RC Disabled"
            joy.axes = [0.0] * 8
            joy.buttons = [0] * 11
            self.pub_joy.publish(joy)
            self.pub_disable = True

    def on_shutdown(self):
        rospy.loginfo("[%s] Shutdown." % (self.node_name))

    def timer_reset_joy(self, event):
        self.joy.axes = [0.0] * 8
        self.joy.buttons = [0] * 11
        self.publish(force=True)
        self.reset = True


if __name__ == "__main__":
    rospy.init_node("input_cmd")
    sendcmd = RC2Joy()
    rospy.on_shutdown(sendcmd.on_shutdown)
    rospy.spin()
