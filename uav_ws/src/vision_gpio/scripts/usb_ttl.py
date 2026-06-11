#!/usr/bin/env python3
import rospy
import serial
from std_msgs.msg import Bool

# ================== 串口配置 ==================
SERIAL_PORT = '/dev/ttyUSB0'   # 改成实际
BAUDRATE = 115200

ser = None

def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        rospy.loginfo(f"串口已打开: {SERIAL_PORT}")
    except Exception as e:
        rospy.logerr(f"串口打开失败: {e}")

# ================== 回调函数 ==================
def gpio_callback(msg):
    global ser

    if msg.data:
        rospy.loginfo("="*50)
        rospy.loginfo("【触发事件】收到gpio_trigger=True")

        if ser is not None and ser.is_open:
            try:
                # 发送点动命令
                ser.write(b'1')
                rospy.loginfo("已发送: '1' → 触发继电器")

            except Exception as e:
                rospy.logerr(f"串口发送失败: {e}")
        else:
            rospy.logwarn("串口未连接")

        rospy.loginfo("="*50)

# ================== 主程序 ==================
if __name__ == '__main__':
    rospy.init_node('esp_relay_controller')

    rospy.loginfo("="*50)
    rospy.loginfo("ESP32继电器控制节点启动")
    rospy.loginfo(f"串口: {SERIAL_PORT}")
    rospy.loginfo("监听话题: gpio_trigger")
    rospy.loginfo("="*50)

    init_serial()

    rospy.Subscriber('gpio_trigger', Bool, gpio_callback)

    rospy.loginfo("等待触发信号...")
    rospy.spin()
