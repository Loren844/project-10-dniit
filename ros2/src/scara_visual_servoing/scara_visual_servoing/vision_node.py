import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import snap7
import struct

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.image_pub = self.create_publisher(Image, '/yolo/detections', 10)
        self.br = CvBridge()
        self.model = YOLO('yolov8n.pt')
        self.cam_x = 0.55
        self.cam_y = 0.0
        self.z_dist = 0.75
        self.focal_length = 554.25

        self.plc = snap7.client.Client()
        self.connect_to_plc()

    def connect_to_plc(self):
        try:
            self.plc.disconnect()
            self.plc.connect('192.168.0.10', 0, 1)
            self.get_logger().info("CONNECTE AU PLC")
        except Exception as e:
            self.get_logger().error(f"ERREUR CONNEXION: {e}")

    def image_callback(self, msg):
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(cv_image, verbose=False)
        
        target_found = False
        best_box = None
        
        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            if cls_id in [32, 47, 49]:
                best_box = box
                target_found = True
                break
                
        if target_found and best_box is not None:
            x_c, y_c = best_box.xywh[0][:2].tolist()
            
            x_cam = (x_c - 320.0) * self.z_dist / self.focal_length
            y_cam = (y_c - 240.0) * self.z_dist / self.focal_length
            
            world_x = self.cam_x - y_cam
            world_y = self.cam_y - x_cam
            world_z = 0.05
            
            data = bytearray(12)
            struct.pack_into('>f', data, 0, world_x)
            struct.pack_into('>f', data, 4, world_y)
            struct.pack_into('>f', data, 8, world_z)
            
            try:
                self.plc.db_write(1, 0, data)
                self.get_logger().info(f"ECRITURE OK: X={world_x:.3f}, Y={world_y:.3f}")
            except Exception as e:
                self.get_logger().error(f"ECHEC ECRITURE: {e}")
                self.connect_to_plc()
        else:
            self.get_logger().warning("YOLO NE VOIT PAS LA CIBLE")
            
        annotated_frame = results[0].plot()
        img_msg = self.br.cv2_to_imgmsg(annotated_frame, "bgr8")
        self.image_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plc.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()