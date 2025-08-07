# --- Imports ---
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import message_filters  # Sync RGB + depth
from std_msgs.msg import String
import torch
from ultralytics import YOLO 
import os
from ament_index_python.packages import get_package_share_directory


# --- Node Definition ---
class YOLOConeDetector3D(Node):
    def __init__(self):
        super().__init__('yolo_cone_detector_3d_node')
        self.bridge = CvBridge()
        
        # Device detection and setup
        self.device = self.setup_device()
        
        # Load YOLO model (path to workspace)
        package_dir = get_package_share_directory('cone_detector')
        model_path = os.path.join(package_dir, 'models', 'best.pt')
        self.model = self.load_model(model_path)
        
        # Model parameters
        self.conf_threshold = 0.4  # Confidence threshold
        self.iou_threshold = 0.45  # IoU threshold for NMS
        self.max_detection_distance = 30.0  # Maximum detection distance in meters
        
        # Image masking parameters 
        self.mask_upper_fraction = 0.4  # Mask upper region of image
        
        # Subscribe to RGB + depth images
        self.rgb_sub = message_filters.Subscriber(self, Image, '/zed/left/image_rect_color')
        self.depth_sub = message_filters.Subscriber(self, Image, '/zed/depth/image_raw')
        
        # Sync both image streams
        self.ts = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.image_callback)
        
        self.get_logger().info("YOLO ConeDetector 3D node started.")
        self.publisher_ = self.create_publisher(String, 'detected_cones', 10)
        
        # Performance tracking
        self.inference_times = []
        self.frame_count = 0
    
    def setup_device(self):
        """Setup computing device with comprehensive GPU detection"""
        try:
            # Check if CUDA available
            if torch.cuda.is_available():
                device_count = torch.cuda.device_count()
                device_name = torch.cuda.get_device_name(0)
                memory_gb = torch.cuda.get_device_properties(0).total_memory / 1e9
                
                self.get_logger().info(f"GPU detected: {device_name}")
                self.get_logger().info(f"GPU memory: {memory_gb:.1f} GB")
                self.get_logger().info(f"Number of GPUs: {device_count}")
                
                device = torch.device('cuda:0')
                
                # Test GPU functionality
                test_tensor = torch.randn(10, 10).to(device)
                _ = test_tensor @ test_tensor.T  # Matrix multiplication test
                
                self.get_logger().info("GPU functionality test passed - using GPU")
                return device
                
            else:
                self.get_logger().warning("CUDA not available")
                
        except Exception as e:
            self.get_logger().error(f"GPU setup failed: {e}")
        
        # Fallback CPU
        self.get_logger().info("Falling back to CPU")
        return torch.device('cpu')
    
    def load_model(self, model_path):
        """Load YOLO model with error handling"""
        try:
            self.get_logger().info(f"Loading model from: {model_path}")
            
            # Load model
            model = YOLO(model_path)
            
            # Move model to device
            model.to(self.device)
            
            # Get model info
            model_info = model.info()
            self.get_logger().info(f"Model loaded successfully")
            self.get_logger().info(f"Model device: {next(model.model.parameters()).device}")
            
            # Class names for debugging
            if hasattr(model.model, 'names'):
                class_names = model.model.names
                self.get_logger().info(f"Model classes: {class_names}")
                
                # Print class mapping
                self.get_logger().info("Class mapping being used:")
                for class_id, name in class_names.items():
                    color, label = self.get_cone_color_from_class(class_id, model)
                    self.get_logger().info(f"  {class_id}: {name} -> {color} (label: {label})")
            
            # Test inference with dummy image
            dummy_image = np.zeros((640, 640, 3), dtype=np.uint8)
            test_results = model(dummy_image, verbose=False)
            self.get_logger().info("Model inference test passed")
            
            return model
            
        except FileNotFoundError:
            self.get_logger().error(f"Model file not found: {model_path}")
            self.get_logger().info("Available files in weights folder:")
            import os
            if os.path.exists('weights'):
                for file in os.listdir('weights'):
                    self.get_logger().info(f"  - {file}")
            raise
            
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            raise
    
    def run_yolo_inference(self, image):
        """Run YOLO inference on the input image"""
        try:
            # Inference with device specification
            results = self.model(image, conf=self.conf_threshold, iou=self.iou_threshold, device=self.device)
            return results[0]  # First (and only) result
            
        except RuntimeError as e:
            if "out of memory" in str(e).lower():
                self.get_logger().error("GPU out of memory! Consider:")
                self.get_logger().error("1. Reducing image size")
                self.get_logger().error("2. Using a smaller model")
                self.get_logger().error("3. Reducing batch size")
                
                # Try to clear GPU cache
                if torch.cuda.is_available():
                    torch.cuda.empty_cache()
                    
            self.get_logger().error(f"YOLO inference failed: {e}")
            return None
            
        except Exception as e:
            self.get_logger().error(f"YOLO inference failed: {e}")
            return None
    
    def parse_yolo_results(self, results):
        """Parse YOLO results into bounding boxes and classes"""
        detections = []
        
        if results is None or results.boxes is None:
            return detections
            
        boxes = results.boxes.cpu().numpy()
        
        for box in boxes:
            # Extract bounding box coordinates
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = float(box.conf[0])
            class_id = int(box.cls[0])
            
            # Convert to x, y, w, h format
            x, y, w, h = x1, y1, x2 - x1, y2 - y1
            
            detections.append({
                'bbox': (x, y, w, h),
                'confidence': confidence,
                'class_id': class_id,
                'center': (x + w // 2, y + h // 2)
            })
        
        return detections
    
    def get_cone_color_from_class(self, class_id, model=None):
        """Map YOLO class ID to cone color and label"""

        # 0: blue_cone, 1: large_orange_cone, 2: orange_cone, 3: unknown_cone, 4: yellow_cone
        class_mapping = {
            0: ("blue", 0),              # blue_cone
            1: ("large_orange", 2),      # large_orange_cone -> treat as orange
            2: ("orange", 2),            # orange_cone
            3: ("unknown", 3),           # unknown_cone -> new class
            4: ("yellow", 1),            # yellow_cone
        }
        
        result = class_mapping.get(class_id, ("unknown", -1))
        
        # Debug logging (only if model available)
        if model and hasattr(model.model, 'names'):
            model_class_name = model.model.names.get(class_id, 'unknown')
            if hasattr(self, 'frame_count') and self.frame_count % 100 == 0:  # Log less frequently
                self.get_logger().debug(f"Class {class_id} ({model_class_name}) -> {result}")
        
        return result
    
    def mask_image_roi(self, image):
        """Mask the upper portion of the image to focus on relevant area"""
        height, width = image.shape[:2]
        mask_height = int(height * self.mask_upper_fraction)
        
        # Create masked image (set upper portion to black)
        masked_image = image.copy()
        masked_image[:mask_height, :] = 0
        
        return masked_image
    
    def image_callback(self, rgb_msg, depth_msg):
        import time
        start_time = time.time()
        
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, '32FC1')
        
        # Apply ROI masking to focus on relevant area
        masked_rgb = self.mask_image_roi(rgb_image)
        
        # Run YOLO inference on masked image
        inference_start = time.time()
        results = self.run_yolo_inference(masked_rgb)
        inference_time = time.time() - inference_start
        
        # Parse results
        detections = self.parse_yolo_results(results)
        
        # Camera intrinsics
        fx, fy = 525.0, 525.0
        cx, cy = rgb_image.shape[1] // 2, rgb_image.shape[0] // 2
        
        message_lines = []  # All cone lines to be published
        
        # Debug: Count detections by class
        class_counts = {}
        
        for detection in detections:
            x, y, w, h = detection['bbox']
            cx_pixel, cy_pixel = detection['center']
            confidence = detection['confidence']
            class_id = detection['class_id']
            
            # Count detections by class for debugging
            class_counts[class_id] = class_counts.get(class_id, 0) + 1
            
            # Get depth at cone center
            depth = float(depth_image[cy_pixel, cx_pixel])
            if np.isnan(depth) or depth == 0.0:
                self.get_logger().debug(f"Skipping detection due to invalid depth: {depth}")
                continue
            
            # Calculate 3D position
            Z = depth
            X = (cx_pixel - cx) * Z / fx
            Y = (cy_pixel - cy) * Z / fy
            
            # Apply distance filter
            if Z > self.max_detection_distance:
                self.get_logger().debug(f"Skipping detection beyond {self.max_detection_distance}m: {Z:.1f}m")
                continue
            
            # Get cone color from class
            colour, label = self.get_cone_color_from_class(class_id, self.model)
            if label == -1:
                self.get_logger().debug(f"Skipping unknown class: {class_id}")
                continue  # Skip unknown classes
            
            message_lines.append(f"{X:.2f},{Y:.2f},{Z:.2f},{label}")
            
            # Get original class name for debugging
            model_class_name = "unknown"
            if hasattr(self.model.model, 'names'):
                model_class_name = self.model.model.names.get(class_id, "unknown")
            
            print(f"Detected: {model_class_name} -> {colour} | Conf={confidence:.2f} | Pos=({X:.2f}, {Y:.2f}, {Z:.2f})")
            
            # Draw bounding box and label
            cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(rgb_image, f"{model_class_name}", 
                       (x, y - 25), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
            cv2.putText(rgb_image, f"{colour} {confidence:.2f}", 
                       (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(rgb_image, f"[{X:.1f}, {Y:.1f}, {Z:.1f}]", 
                       (x, y + h + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        
        # Debug: Print class distribution every 50 frames
        if self.frame_count % 50 == 0 and class_counts:
            self.get_logger().info(f"Detection class distribution: {class_counts}")
        
        # Publish message
        if message_lines:
            msg = String()
            msg.data = "\n".join(message_lines)
            self.publisher_.publish(msg)
        
        # Performance tracking
        total_time = time.time() - start_time
        self.inference_times.append(inference_time)
        self.frame_count += 1
        
        # Log performance every 100 frames
        if self.frame_count % 100 == 0:
            avg_inference_time = np.mean(self.inference_times[-100:])
            fps = 1.0 / total_time if total_time > 0 else 0
            self.get_logger().info(f"Avg inference time: {avg_inference_time:.3f}s, FPS: {fps:.1f}, Detections: {len(detections)}")
        
        # Draw ROI mask boundary for visualisation
        height = rgb_image.shape[0]
        mask_line_y = int(height * self.mask_upper_fraction)
        cv2.line(rgb_image, (0, mask_line_y), (rgb_image.shape[1], mask_line_y), (255, 0, 0), 2)
        cv2.putText(rgb_image, "ROI Mask", (10, mask_line_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
        
        # Display image
        cv2.imshow("YOLO Cone Detection 3D", rgb_image)
        cv2.waitKey(1)

# --- Entry Point ---
def main(args=None):
    rclpy.init(args=args)
    node = YOLOConeDetector3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()