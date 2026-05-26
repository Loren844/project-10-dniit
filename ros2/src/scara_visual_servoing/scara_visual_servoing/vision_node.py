import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        
        # Abonnement au flux de la caméra simulée
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
            
        self.br = CvBridge()
        # Charger ton modèle YOLO (utilise yolov8n.pt pour aller vite)
        self.model = YOLO('yolov8n.pt') 
        self.get_logger().info("Nœud de vision YOLO démarré !")

    def image_callback(self, msg):
        # 1. Convertir le message ROS en image OpenCV
        cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # 2. Inférence YOLO
        results = self.model(cv_image, verbose=False)
        
        # 3. Dessiner les boîtes (Optionnel, pour le debug)
        annotated_frame = results[0].plot()
        cv2.imshow("Gazebo Camera - YOLOv8", annotated_frame)
        cv2.waitKey(1)
        
        # 4. Extraire le centre de l'objet détecté (Ex: une tasse ou ta boule rouge)
        for box in results[0].boxes:
            # Récupérer les coordonnées (x_center, y_center) en pixels
            x_c, y_c = box.xywh[0][:2].tolist()
            # Ici tu peux envoyer ces coordonnées à ton nœud d'asservissement visuel !

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()