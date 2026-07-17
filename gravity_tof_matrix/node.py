import os
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

import serial


class LidarNode(Node):
    def __init__(self):
        super().__init__('gravity_tof_matrix_node')

        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('topic', '/points')

        serial_port = self.get_parameter('port').value
        topic_name = self.get_parameter('topic').value

        self.publisher = self.create_publisher(PointCloud2, topic_name, 10)
        
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.get_logger().info(f"Connected to Lidar on {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
            sys.exit(1)

        set_mode_pkt = bytes([0x55, 0x00, 0x05, 0x01, 0x00, 0x00, 0x00, 0x03])
        self.get_logger().debug("Sending initialization packet to wake up LiDAR...")
        self.ser.write(set_mode_pkt)
        time.sleep(0.3) 
        
        if self.ser.in_waiting:
            self.ser.read(self.ser.in_waiting)

        self.request_data_pkt = bytes([0x55, 0x00, 0x01, 0x02])

        self.create_timer(0.05, self.update_lidar)

    def update_lidar(self):
        try:
            self.ser.write(self.request_data_pkt)
            
            header = self.ser.read(4)
            if len(header) != 4:
                return

            status, echo_cmd, len_l, len_h = header
            
            if status == 0x53 and echo_cmd == 0x02:
                data_len = (len_h << 8) | len_l
                
                if data_len == 128:
                    payload = self.ser.read(128)
                    if len(payload) == 128:
                        self.process_and_publish(payload)
                        
        except Exception as e:
            self.get_logger().warn(f"Serial communication error: {e}")

    def process_and_publish(self, payload):

        distances_mm = np.frombuffer(payload, dtype=np.uint16)
        distances_m = distances_mm.astype(np.float32) / 1000.0

        points = np.zeros((64, 3), dtype=np.float32)
        
        for i in range(64):
            row = i // 8
            col = i % 8
            
            points[i, 0] = col * 0.05    # X axis
            points[i, 1] = row * 0.05    # Y axis
            points[i, 2] = distances_m[i] # Z axis

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        
        msg.point_step = 12
        msg.row_step = msg.point_step * 64
        msg.is_dense = True
        msg.height = 1
        msg.width = 64
        msg.data = points.tobytes()

        self.publisher.publish(msg)

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()