import rclpy
from rclpy.node import Node
from control.matlab import *
from rclpy.clock import Clock

from std_msgs.msg import Float32
import smbus

class Median:
    def __init__(self,l):
        self.array = [0]*l
    def get_median(self,data):
        self.array.pop(0)
        self.array.append((sum(self.array) + data)/(len(self.array)+1))
        return sum(self.array)/len(self.array)

class Subscriber(Node):

    def __init__(self):
        self.filtr = Median(3)
        self.prev_ang = 0
        self.k = 0
        self.bus = smbus.SMBus(0)
        self.prev_t = Clock().now().seconds_nanoseconds()
        
        super().__init__('i2c_read')
        self.publisher_ = self.create_publisher(Float32, 'pos_wheel', 10)
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.pos_wheel)
        timer_period2 = 0.04
        self.timer2 = self.create_timer(timer_period2, self.pub)

    def pub(self):
        time = Clock().now().seconds_nanoseconds()
        dt = time[0] - self.prev_t[0] + time[1]/(10**9) - self.prev_t[1]/(10**9)

        data2=Float32()
        data2.data =self.filtr.get_median(self.k/dt/4)
        self.publisher_.publish(data2)
        self.k = 0
        self.prev_t = time
        
    def pos_wheel(self):
        data1 = self.bus.read_byte_data(0x36,0x0D)
        data2 = self.bus.read_byte_data(0x36,0x0C)
        data = data2 << 8 | data1
        print(data*0.087890625,self.prev_ang)
        print(data*0.087890625 - self.prev_ang < 0)

        if 0 < data*0.087890625 < 90 and 270 < self.prev_ang < 360:
            self.k+=1
        elif 90 < data*0.087890625 < 180 and 0 < self.prev_ang < 90:
            self.k+=1
        elif 180 < data*0.087890625 < 270 and 90 < self.prev_ang < 180:
            self.k+=1
        elif 270 < data*0.087890625 < 360 and 180 < self.prev_ang < 270:
            self.k+=1
        self.prev_ang = data*0.087890625

        

        



        
        

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