import rclpy
from rclpy.node import Node
import json
from std_msgs.msg import String
import time


class ControlSystem(Node):

    def __init__(self):
        super().__init__('control_system')
        #create subcription to serial_rc topic
        self.serial_rc_subscription = self.create_subscription(
            String,
            'serial_rc',
            self.serial_rc_listener_callback,
            10)
        self.serial_rc_subscription
        
        #create subscription to airmar_data
        self.airmar_data_subscription = self.create_subscription(
            String,
            'airmar_data',
            self.airmar_data_listener_callback,
            10)
        self.airmar_data_subscription
        
        #create subscription to teensy_status
        self.teensy_status_subscription = self.create_subscription(
            String,
            'teensy_status',
            self.teensy_status_listener_callback,
            10)
        self.teensy_status_subscription
        
        #create publisher to pwm_control
        self.pwm_control_publisher_ = self.create_publisher(String, 'pwm_control', 10)

        #create publisher to teensy_control
        self.teensy_control_publisher_ = self.create_publisher(String, 'teensy_control', 10)

        #create instance vars for subscribed topics to update
        self.serial_rc = {}
        self.airmar_data = {}
        self.teensy_status = {}

        #create instance var for keeping queue of wind data
        self.lastWinds = []
        

    def serial_rc_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.serial_rc[i] = msg_dict[i]
        
    def airmar_data_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.airmar_data[i] = msg_dict[i]
        
    def teensy_status_listener_callback(self, msg):
        self.get_logger().info('Received msg: "%s"' % msg.data)
        msg_dict = json.loads(msg.data)
        for i in msg_dict:
            self.teensy_status[i] = msg_dict[i]


    def findTrimTabState(self, relativeWind):
        #check we have new wind
        if(relativeWind == self.lastWinds[len(self.lastWinds) -1]):
            return       
        #first add wind to running list
        self.lastWinds.append(relativeWind)
        if(len(self.lastWinds) > 10):
            lastWinds.pop(0)
        #now find best trim tab state
        smoothAngle = self.median(self.lastWinds)
        if(smoothAngle >= 45.0 and smoothAngle < 90):
            #max lift starboard
            toPub = makeJsonString({"state":"1"})
            self.teensy_control_publisher_.publish(toPub)
        elif(smoothAngle >= 90 and smoothAngle < 180):
            #max drag starboard
            toPub = makeJsonString({"state":"3"})
            self.teensy_control_publisher_.publish(toPub)
        elif(smoothAngle >= 180 and smoothAngle < 270):
            #max drag port
            toPub = makeJsonString({"state":"2"})
            self.teensy_control_publisher_.publish(toPub)
        elif(smoothAngle >= 270 and smoothAngle < 315):
            #max lift port
            toPub = makeJsonString({"state":"0"})
            self.teensy_control_publisher_.publish(toPub)
        else:
            #in irons, min lift
            toPub = makeJsonString({"state":"4"})
            self.teensy_control_publisher_.publish(toPub)
        

            
    def makeJsonString(self, jsonMsg):
        json_str = json.dumps(jsonMsg)
        message = String()
        message.data = json_str
        return message

           
    def median(self, lst):
        n = len(lst)
        s = sorted(lst)
        return (sum(s[n//2-1:n//2+1])/2.0, s[n//2])[n % 2] if n else None

           
def main(args=None):
    rclpy.init(args=args)

    control_system = ControlSystem()
    
    while( rclpy.ok() ):
        
        rclpy.spin_once(control_system, timeout_sec=.5)
        # now we have new vals from subscribers in:
        # control_system.serial_rc
        # control_system.airmar_data
        # control_system.teensy_status

        # need to publish new values to both control topics based on new values
        # control_system.pwm_control_publisher_.publish()
        # control_system.teensy_control_publisher_.publish()

        #TODO ^^implement
        
        inRC = True
        if(len(control_system.serial_rc) < 2):
            pass #don't have rc values
        elif(inRC):
            if(float(control_system.serial_rc["state1"]) < 400):
                #manual
                manualAngle = int((float(control_system.serial_rc["manual"]) / 2000) * 86) + 72
                toPub = control_system.makeJsonString({"state":"5","angle":manualAngle})
                control_system.teensy_control_publisher_.publish(toPub)
            elif("wind-angle-relative" in control_system.airmar_data ):
		#print(control_system.airmar_data["wind-angle-relative"])
                control_system.findTrimTabState(control_system.airmar_data["wind-angle-relative"])
            else:
                print("No wind angle values")
            rudderAngle = (float(control_system.serial_rc["rudder"]) / 2000 * 90) + 25
            rudderJson = {"channel" : "8", "angle" : rudderAngle}
            control_system.pwm_control_publisher_.publish(control_system.makeJsonString(rudderJson))
            ballastAngle = 0
            if(control_system.serial_rc["ballast"] > 1200):
                ballastAngle = 110
            elif(control_system.serial_rc["ballast"] < 800):
                ballastAngle = 80
            ballastJson = {"channel" : "12", "angle" : ballastAngle}
            control_system.pwm_control_publisher_.publish(control_system.makeJsonString(ballastJson))
        


    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    control_system.destroy_node()
    rclpy.shutdown()








if __name__ == '__main__':
    main()
