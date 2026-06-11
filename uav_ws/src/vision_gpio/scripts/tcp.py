#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import socket
import rospy
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

PORT = 8080
BUFFER_SIZE = 1024

client_socket_global = None
land_done_flag = False


def parse_data(data_str):
    parts = data_str.strip().split()

    if len(parts) != 8:
        raise ValueError("Invalid data format")

    name1 = parts[0]
    x1 = float(parts[1])
    y1 = float(parts[2])
    z1 = float(parts[3])

    name2 = parts[4]
    x2 = float(parts[5])
    y2 = float(parts[6])
    z2 = float(parts[7])

    return (name1, x1, y1, z1, name2, x2, y2, z2)


def land_done_callback(msg):
    global client_socket_global, land_done_flag

    if msg.data:
        if not land_done_flag:
            land_done_flag = True

            rospy.loginfo("Received land_done = true")

            if client_socket_global is not None:
                try:
                    client_socket_global.sendall(b"mission_done\n")
                    rospy.loginfo("Sent TCP: mission_done")
                except Exception as e:
                    rospy.logerr("Send done failed: %s", e)


def main():

    global client_socket_global

    rospy.init_node("tcp_to_ros_bridge")

    search_pub = rospy.Publisher(
        "/mission/search_point", Point, queue_size=1, latch=True
    )

    drop_pub = rospy.Publisher(
        "/mission/drop_point", Point, queue_size=1, latch=True
    )

    rtl_pub = rospy.Publisher(
        "/mission/rtl", Bool, queue_size=1, latch=True
    )

    rospy.Subscriber("/land_done", Bool, land_done_callback)

    rospy.loginfo("TCP server starting...")

    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", PORT))
    server_socket.listen(5)

    rospy.loginfo("Server listening on port %d", PORT)

    while not rospy.is_shutdown():

        try:
            rospy.loginfo("Waiting for client...")
            client_socket, addr = server_socket.accept()

            client_socket.settimeout(None)

            client_socket_global = client_socket

            rospy.loginfo("Client connected: %s:%d", addr[0], addr[1])

            buffer = ""

            while not rospy.is_shutdown():

                try:
                    data = client_socket.recv(BUFFER_SIZE)

                    if not data:
                        rospy.logwarn("Client disconnected")
                        break

                    rospy.logwarn("RAW BYTES: %r", data)

                    data_str = data.decode("utf-8", errors="ignore")
                    data_str = data_str.replace("\x00", "")

                    if data_str.strip() == "ping":
                        rospy.loginfo("<<< Heartbeat received")
                        continue

                    rospy.loginfo("TCP Clean Data: %s", data_str)

                    buffer += data_str

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)

                        if not line.strip():
                            continue

                        # 返航指令
                        if line.strip() == "返航":
                            rospy.logwarn("⚠️ 收到返航指令！执行RTL")

                            rtl_msg = Bool()
                            rtl_msg.data = True
                            rtl_pub.publish(rtl_msg)

                            continue

                        try:
                            name1, x1, y1, z1, name2, x2, y2, z2 = parse_data(line)

                        except Exception as e:
                            rospy.logerr("Parse failed: %s | Data: %s", e, line)
                            continue

                        search_point = Point()
                        search_point.x = x1
                        search_point.y = y1
                        search_point.z = z1

                        drop_point = Point()
                        drop_point.x = x2
                        drop_point.y = y2
                        drop_point.z = z2

                        rospy.loginfo(
                            "Search Point: %.3f %.3f %.3f",
                            search_point.x, search_point.y, search_point.z
                        )

                        rospy.loginfo(
                            "Drop Point: %.3f %.3f %.3f",
                            drop_point.x, drop_point.y, drop_point.z
                        )

                        search_pub.publish(search_point)
                        drop_pub.publish(drop_point)

                        try:
                            client_socket.sendall(b"ack\n")
                            rospy.loginfo("ACK sent: ack")
                        except Exception as e:
                            rospy.logerr("ACK send failed: %s", e)

                except Exception as e:
                    rospy.logerr("Socket error: %s", e)
                    break

        except Exception as e:
            rospy.logerr("Connection error: %s", e)

        finally:
            try:
                if client_socket_global is not None:
                    client_socket_global.close()
            except:
                pass

            client_socket_global = None
            rospy.loginfo("Ready for next client...")

    server_socket.close()


if __name__ == "__main__":
    main()