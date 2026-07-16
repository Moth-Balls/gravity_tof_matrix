import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import numpy as np
import serial
import sys
import threading

class LidarNode(Node):
    def __init__(self):
        super().__init__('gravity_tof_matrix_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('topic', '/points')

        serial_port = self.get_parameter('port').value
        topic_name = self.get_parameter('topic').value

        self.publisher = self.create_publisher(PointCloud2, topic_name, 10)
        
        self.matrix = np.zeros((8, 8), dtype=np.float32)
        
        try:
            self.ser = serial.Serial(serial_port, 115200, timeout=1)
            self.ser.reset_input_buffer()
            self.get_logger().info(f"Connected directly to Lidar on {serial_port}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {serial_port}: {e}")
            sys.exit(1)

        self.running = True
        self.read_thread = threading.Thread(target=self.serial_listener, daemon=True)
        self.read_thread.start()

    def serial_listener(self):
        while self.running and rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    
                    if line.startswith('y') and ':' in line:
                        row_part, data_part = line.split(':')
                        
                        row_idx = int(row_part[1])
                        
                        distances = [float(x) / 1000.0 for x in data_part.split(',') if x]
                        
                        if len(distances) == 8 and 0 <= row_idx < 8:
                            self.matrix[row_idx] = distances
                            
                            if row_idx == 7:
                                self.publish_scan()
                                
            except Exception as e:
                self.get_logger().warn(f"Error parsing serial data: {e}")

    def publish_scan(self):
        """Converts the parsed 8x8 matrix into a PointCloud2 and publishes it."""
        points = np.zeros((64, 3), dtype=np.float32)
        
        for row in range(8):
            for col in range(8):
                idx = row * 8 + col
                dist = self.matrix[row, col]
                
                # Convert grid indexes to x, y, z coordinates
                points[idx, 0] = col * 0.1
                points[idx, 1] = row * 0.1
                points[idx, 2] = dist

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
        self.running = False
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