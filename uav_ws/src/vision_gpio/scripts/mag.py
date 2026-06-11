#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial
from std_msgs.msg import Bool

# ================== 串口配置 ==================
SERIAL_PORT = '/dev/ttyUSB0'   
BAUDRATE = 9600 

ser = None
is_triggering = False  # 状态锁，防止 200ms 内被重复触发导致时序混乱

# ================== 继电器控制指令 ==================
# 第 1 路开关
RELAY_ON_CMD = bytes.fromhex('A0 01 01 A2')
RELAY_OFF_CMD = bytes.fromhex('A0 01 00 A1')

def init_serial():
    global ser
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
        rospy.loginfo(f"串口已打开: {SERIAL_PORT}, 波特率: {BAUDRATE}")
    except Exception as e:
        rospy.logerr(f"串口打开失败: {e}")

# ================== 回调函数 ==================
def gpio_callback(msg):
    global ser, is_triggering

    # 如果收到 False (虽然你说不会发，但加个判断可以防误触)，直接忽略
    if not msg.data:
        return

    # 如果当前正在执行 200ms 的点动动作，忽略新的触发信号
    if is_triggering:
        rospy.logwarn("继电器正在动作中，忽略本次重复触发")
        return

    if ser is not None and ser.is_open:
        try:
            is_triggering = True  # 上锁
            rospy.loginfo("="*50)
            rospy.loginfo("【触发】收到 True，开始 200ms 供电")
            
            # 1. 继电器闭合（通电）
            ser.write(RELAY_ON_CMD)
            
            # 2. 延时 200ms (0.2秒)
            rospy.sleep(0.2) 
            
            # 3. 继电器断开（断电）
            ser.write(RELAY_OFF_CMD)
            
            rospy.loginfo("【完成】已通电 200ms 并自动断开")
            rospy.loginfo("="*50)

        except Exception as e:
            rospy.logerr(f"串口通信出错: {e}")
        finally:
            is_triggering = False  # 无论成功或报错，最后都解锁
    else:
        rospy.logwarn("串口未连接，无法执行动作")

# ================== 主程序 ==================
if __name__ == '__main__':
    rospy.init_node('usb_relay_jog_controller')

    rospy.loginfo("="*50)
    rospy.loginfo(f"串口: {SERIAL_PORT} | 波特率: {BAUDRATE}")
    rospy.loginfo("="*50)

    init_serial()

    # 订阅 gpio_trigger 话题
    rospy.Subscriber('gpio_trigger', Bool, gpio_callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 节点关闭时确保继电器断电并释放串口
        if ser is not None and ser.is_open:
            try:
                ser.write(RELAY_OFF_CMD)
                ser.close()
                rospy.loginfo("已安全断电并关闭串口")
            except:
                pass
