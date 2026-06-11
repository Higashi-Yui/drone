#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
import sys

class Colors:
    HEADER = '\033[95m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    GREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

def get_float_input(prompt, default_val=None, min_val=None):
    
    while not rospy.is_shutdown():
        try:
            val_str = input(prompt).strip()
            
            # 处理退出指令
            if val_str.lower() == 'q':
                return None
                
            # 处理默认值
            if val_str == '' and default_val is not None:
                val = float(default_val)
            else:
                val = float(val_str)
                
            # 处理最小值过滤 (安全拦截)
            if min_val is not None and val <= min_val:
                print(Colors.FAIL + f"  ✖ 安全限制触发：该高度数值过低 (必须 > {min_val}m)，请重新输入！" + Colors.ENDC)
                continue
                
            return val
            
        except ValueError:
            print(Colors.FAIL + "  ✖ 无效输入！请输入数字，或输入 'q' 取消。" + Colors.ENDC)
        except KeyboardInterrupt:
            return None

def print_menu():
    print(Colors.HEADER + Colors.BOLD + "\n=================================================" + Colors.ENDC)
    print(Colors.CYAN + Colors.BOLD + "     FY-330物流无人机任务下发终端" + Colors.ENDC)
    print(Colors.HEADER + Colors.BOLD + "=================================================\n" + Colors.ENDC)
    print(Colors.BLUE + "  [1] " + Colors.ENDC + "手动配置任务 (坐标 & 高度)")
    print(Colors.BLUE + "  [2] " + Colors.ENDC + "发送预设测试点")
    print(Colors.FAIL + "  [3] " + Colors.ENDC + "触发紧急返航 (RTL)")
    print(Colors.BLUE + "  [0] " + Colors.ENDC + "退出程序\n")

def main():
    rospy.init_node('mission_waypoint_terminal', anonymous=True)
    
    search_pub = rospy.Publisher('/mission/search_point', Point, queue_size=10)
    drop_pub = rospy.Publisher('/mission/drop_point', Point, queue_size=10)
    rtl_pub = rospy.Publisher('/mission/rtl', Bool, queue_size=10)
    
    rospy.sleep(0.5)

    # 高度安全底线设定
    SAFE_MIN_Z = 0.1 

    while not rospy.is_shutdown():
        print_menu()
        choice = input(Colors.BOLD + "请输入操作序号: " + Colors.ENDC).strip()

        if choice == '1':
            print(Colors.CYAN + "\n--- 设置搜寻点 & 巡航高度 ---" + Colors.ENDC)
            s_x = get_float_input(" ├── 搜寻点 X 坐标 (m): ")
            if s_x is None: continue
            s_y = get_float_input(" ├── 搜寻点 Y 坐标 (m): ")
            if s_y is None: continue
            
            # 加入 min_val 限制
            c_z = get_float_input(f" └── 巡航高度 (m) {Colors.BLUE}{Colors.ENDC}: ", default_val=1.3, min_val=SAFE_MIN_Z)
            if c_z is None: continue

            print(Colors.CYAN + "\n--- 设置投放点 & 抓取高度 ---" + Colors.ENDC)
            d_x = get_float_input(" ├── 投放点 X 坐标 (m): ")
            if d_x is None: continue
            d_y = get_float_input(" ├── 投放点 Y 坐标 (m): ")
            if d_y is None: continue
            
            # 加入 min_val 限制
            g_z = get_float_input(f" └── 抓取高度 (m) {Colors.BLUE}{Colors.ENDC}: ", default_val=0.86, min_val=SAFE_MIN_Z)
            if g_z is None: continue

            print(Colors.WARNING + f"\n请确认飞行参数:\n  搜寻任务: X={s_x}, Y={s_y} | 巡航高度(Cruise)={c_z}m\n  抓取任务: X={d_x}, Y={d_y} | 抓取高度(Grab)={g_z}m" + Colors.ENDC)
            
            confirm = input(Colors.BOLD + "确认下发任务吗？(y/n): " + Colors.ENDC).strip().lower()
            if confirm == 'y':
                search_pub.publish(Point(x=s_x, y=s_y, z=c_z))
                drop_pub.publish(Point(x=d_x, y=d_y, z=g_z - 0.04))
                print(Colors.GREEN + Colors.BOLD + "\n✔ 任务参数已成功发送！" + Colors.ENDC)
            else:
                print(Colors.FAIL + "✖ 操作已取消。" + Colors.ENDC)

        elif choice == '2':
            s_x, s_y, c_z = 3.0, 0.0, 1.3
            d_x, d_y, g_z = 5.0, 2.0, 0.86
            print(Colors.WARNING + f"\n将发送预设参数:\n  搜寻: ({s_x}, {s_y}) 高度: {c_z}m\n  抓取: ({d_x}, {d_y}) 高度: {g_z}m" + Colors.ENDC)
            confirm = input(Colors.BOLD + "确认下发吗？(y/n): " + Colors.ENDC).strip().lower()
            if confirm == 'y':
                search_pub.publish(Point(x=s_x, y=s_y, z=c_z))
                drop_pub.publish(Point(x=d_x, y=d_y, z=g_z))
                print(Colors.GREEN + Colors.BOLD + "\n✔ 预设任务已发送！" + Colors.ENDC)
            else:
                print(Colors.FAIL + "✖ 操作已取消。" + Colors.ENDC)

        elif choice == '3':
            print(Colors.FAIL + Colors.BOLD + "\n！！！ 警告：即将触发紧急返航 (RTL) ！！！" + Colors.ENDC)
            confirm = input(Colors.FAIL + "确认紧急返航？(y/n): " + Colors.ENDC).strip().lower()
            if confirm == 'y':
                rtl_pub.publish(Bool(data=True))
                print(Colors.FAIL + Colors.BOLD + "✔ RTL 指令已广播！" + Colors.ENDC)
            else:
                print(Colors.GREEN + "✖ 已取消。" + Colors.ENDC)

        elif choice == '0' or choice.lower() == 'q':
            break

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        sys.exit(0)