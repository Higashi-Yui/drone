#!/usr/bin/env python3
import rospy
import subprocess
from std_msgs.msg import Bool
from std_msgs.msg import String


rospy.init_node('gpio_pub_sub')
pub = rospy.Publisher('gpio', String, queue_size=10)

# GPIO端口定义
GPIO4_A3_PIN = '3'  # GPIO4_A3对应的wiringPi编号
GPIO4_A4_PIN = '4'  # GPIO4_A4对应的wiringPi编号

# 打印初始信息
rospy.loginfo("="*50)
rospy.loginfo("GPIO控制程序启动")
rospy.loginfo(f"GPIO4_A3 (wiringPi pin {GPIO4_A3_PIN}): 设置为输入模式")
rospy.loginfo(f"GPIO4_A4 (wiringPi pin {GPIO4_A4_PIN}): 设置为输入模式")
rospy.loginfo("="*50)

# 设置GPIO4_A3为输入
subprocess.run(['gpio', 'mode', GPIO4_A3_PIN, 'in'])
# 设置GPIO4_A4默认输入
subprocess.run(['gpio', 'mode', GPIO4_A4_PIN, 'in'])

def gpio_callback(msg):
    if msg.data :
        rospy.loginfo("="*50)
        rospy.loginfo("【触发事件】收到触发信号，GPIO4_A4输出高电平200ms")
        rospy.loginfo(f"GPIO4_A4 (wiringPi pin {GPIO4_A4_PIN}): 设置为输出模式")
        
        # 设置为输出模式
        subprocess.run(['gpio', 'mode', GPIO4_A4_PIN, 'out'])
        # 输出高电平
        subprocess.run(['gpio', 'write', GPIO4_A4_PIN, '1'])
        rospy.loginfo(f"GPIO4_A4 (wiringPi pin {GPIO4_A4_PIN}): 输出高电平")
        
        # 延时200ms
        rospy.sleep(0.2)
        
        # 恢复低电平
        subprocess.run(['gpio', 'write', GPIO4_A4_PIN, '0'])
        rospy.loginfo(f"GPIO4_A4 (wiringPi pin {GPIO4_A4_PIN}): 恢复低电平")
        
        # 恢复输入模式
        subprocess.run(['gpio', 'mode', GPIO4_A4_PIN, 'in'])
        rospy.loginfo(f"GPIO4_A4 (wiringPi pin {GPIO4_A4_PIN}): 恢复输入模式")
        rospy.loginfo("GPIO4_A4已恢复")
        rospy.loginfo("="*50)

# 订阅触发话题
rospy.Subscriber('gpio_trigger', Bool, gpio_callback)

rospy.loginfo("正在监听 gpio_trigger 话题...")
rospy.loginfo("等待触发信号...")

rate = rospy.Rate(30)
while not rospy.is_shutdown():
    a3 = subprocess.run(['gpio', 'read', GPIO4_A3_PIN], capture_output=True, text=True).stdout.strip()
    a4 = subprocess.run(['gpio', 'read', GPIO4_A4_PIN], capture_output=True, text=True).stdout.strip()
    
    # 每30次循环（约1秒）打印一次状态
    if rospy.get_time() % 1.0 < 0.03:  # 大约每秒打印一次
        rospy.loginfo(f"[状态] GPIO4_A3={a3}, GPIO4_A4={a4}")
    
    pub.publish(f"{a3},{a4}")
    rate.sleep()

rospy.loginfo("GPIO控制程序结束")
