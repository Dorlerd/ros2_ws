import rclpy
from rclpy.node import Node
from control.matlab import *
from rclpy.clock import Clock
import math
from geometry_msgs.msg import TransformStamped

from std_msgs.msg import Float32
from geometry_msgs.msg import Vector3
from tf2_ros import TransformBroadcaster
import smbus

class Differ():

    def __init__(self):
        self.prev_pos = 0

    def data(self,new,time):
        data = (new - self.prev_pos)/time
        self.prev_pos = new
        return data
def grad_to_deg(phi):
    return 3.14/180*phi

def ust_to_d_phi(ust):
    if ust == 0:
        return 0 
    return 3.14/2 - math.atan(1/(math.tan(ust)) - 0.125/(2*0.174))




class Subscriber(Node):

    def __init__(self):
        self.bus = smbus.SMBus(0)

        self.r_wheel = 0.0235

        self.prev_t = Clock().now().seconds_nanoseconds()

        self.ust_rul = 0
        
        self.x = 0
        self.y = 0
        self.phi = 0
        self.d_pos_wheel = Differ()

        super().__init__('wireless_odom')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publisher_ = self.create_publisher(Vector3, 'odom', 10)
        self.subscription = self.create_subscription(Float32,
            '/pos_wheel',
            self.wireless_odom,1)
        self.kalman = self.create_subscription(Vector3,
            '/kalman',
            self.up_date_odom,1)
        self.subscription = self.create_subscription(Vector3,
            '/move_jetracer',
            self.move,1)
        
    def move(self,msg):
        self.ust_rul = msg.y*15


    def wireless_odom(self,msg):
        time = Clock().now().seconds_nanoseconds()
        dt = time[0] - self.prev_t[0] + time[1]/(10**9) - self.prev_t[1]/(10**9)
        
        pos_wheel = msg.data
        v_pos_wheel = self.r_wheel*pos_wheel*2*3.14

        self.x += v_pos_wheel*math.cos(self.phi)*dt
        self.y += v_pos_wheel*math.sin(self.phi)*dt
        self.phi += math.tan(ust_to_d_phi(grad_to_deg(self.ust_rul)))* v_pos_wheel / 0.174 * dt

        data = Vector3()
        data.x = self.x
        data.y = self.y
        data.z = self.phi

        self.publisher_.publish(data)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = self.phi

        self.tf_broadcaster.sendTransform(t)

        self.prev_t = time

    def up_date_odom(self,msg):
        self.x = msg.x
        self.y = msg.y
        self.phi = msg.z

        



        
        

def main(args=None):
    rclpy.init(args=args)

    subscriber = Subscriber()

    rclpy.spin(subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()