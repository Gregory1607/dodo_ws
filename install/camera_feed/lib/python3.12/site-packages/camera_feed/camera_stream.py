import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import argparse
import threading
import queue

class LowLatencyCameraStream(Node):
    def __init__(self, front_ip, rear_ip):
        super().__init__('low_latency_camera_stream')
        
        self.front_pub = self.create_publisher(Image, 'camera_feed/front', 1)
        self.rear_pub = self.create_publisher(Image, 'camera_feed/rear', 1)
        
        self.front_queue = queue.Queue(maxsize=1)
        self.rear_queue = queue.Queue(maxsize=1)
        
        self.cv_bridge = CvBridge()
        
        threading.Thread(target=self.capture_frames, args=(front_ip, self.front_queue), daemon=True).start()
        threading.Thread(target=self.capture_frames, args=(rear_ip, self.rear_queue), daemon=True).start()
        
        self.timer = self.create_timer(0.2, self.publish_frames)  # 5 FPS

    def capture_frames(self, camera_ip, frame_queue):
        cap = cv2.VideoCapture(f'http://{camera_ip}/video')
        if not cap.isOpened():
            self.get_logger().error(f'Failed to open camera stream: {camera_ip}')
            return
        
        frame_count = 0
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                self.get_logger().warn(f'Failed to capture frame from {camera_ip}')
                continue
            
            frame_count += 1
            if frame_count % 3 != 0:  # Skip 2 out of every 3 frames
                continue
            
            frame = cv2.resize(frame, (320, 240))  # Further reduce resolution
            
            try:
                frame_queue.put(frame, block=False)
            except queue.Full:
                pass  # Skip frame if queue is full
        
        cap.release()

    def publish_frames(self):
        try:
            front_frame = self.front_queue.get(block=False)
            front_msg = self.cv_bridge.cv2_to_imgmsg(front_frame, encoding="bgr8")
            self.front_pub.publish(front_msg)
        except queue.Empty:
            pass

        try:
            rear_frame = self.rear_queue.get(block=False)
            rear_msg = self.cv_bridge.cv2_to_imgmsg(rear_frame, encoding="bgr8")
            self.rear_pub.publish(rear_msg)
        except queue.Empty:
            pass

def main(args=None):
    parser = argparse.ArgumentParser(description='Low Latency ROS2 Dual Camera Stream')
    parser.add_argument('--front_ip', type=str, required=True, help='IP address and port of the front camera')
    parser.add_argument('--rear_ip', type=str, required=True, help='IP address and port of the rear camera')
    cli_args = parser.parse_args()

    rclpy.init(args=args)
    node = LowLatencyCameraStream(cli_args.front_ip, cli_args.rear_ip)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

