import json
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
try:
    from ultralytics import YOLO
except Exception:
    YOLO = None
import numpy as np


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('detection_topic', '/detections')
        self.declare_parameter('publish_image', True)
        self.declare_parameter('conf_threshold', 0.25)

        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        detection_topic = self.get_parameter('detection_topic').get_parameter_value().string_value
        self.publish_image = self.get_parameter('publish_image').get_parameter_value().bool_value
        self.conf_threshold = self.get_parameter('conf_threshold').get_parameter_value().double_value

        self.bridge = CvBridge()

        if YOLO is None:
            self.get_logger().error('ultralytics YOLO package not found. Install `ultralytics` to use this node.')
            raise RuntimeError('ultralytics not installed')

        self.get_logger().info(f'Loading model: {model_path}')
        self.model = YOLO(model_path)

        self.sub = self.create_subscription(Image, input_topic, self.listener_callback, 10)
        self.pub_det = self.create_publisher(String, detection_topic, 10)
        if self.publish_image:
            self.pub_img = self.create_publisher(Image, '/yolo/image', 10)

        self.get_logger().info(f'YoloNode subscribed to `{input_topic}` -> publishing detections on `{detection_topic}`')

    def listener_callback(self, msg: Image):
        now = time.time()
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert Image msg to cv2: {e}')
            return

        # Run inference (synchronous)
        try:
            results = self.model(frame)
            res = results[0]
        except Exception as e:
            self.get_logger().error(f'YOLO inference failed: {e}')
            return

        detections = []
        boxes = getattr(res, 'boxes', None)
        if boxes is not None:
            # try extracting xyxy, conf, cls
            try:
                xyxy = boxes.xyxy.cpu().numpy()
                confs = boxes.conf.cpu().numpy()
                cls_inds = boxes.cls.cpu().numpy().astype(int)
            except Exception:
                # fallback: some ultralytics versions use boxes.data
                try:
                    data = boxes.data.cpu().numpy()
                    # data columns: x1,y1,x2,y2,conf,class
                    xyxy = data[:, :4]
                    confs = data[:, 4]
                    cls_inds = data[:, 5].astype(int)
                except Exception:
                    xyxy = np.array([])
                    confs = np.array([])
                    cls_inds = np.array([])

            for box, conf, cls_i in zip(xyxy, confs, cls_inds):
                if conf < self.conf_threshold:
                    continue
                name = self.model.names[int(cls_i)] if hasattr(self.model, 'names') else str(int(cls_i))
                detections.append({
                    'class': name,
                    'confidence': float(conf),
                    'bbox': [float(box[0]), float(box[1]), float(box[2]), float(box[3])]
                })

        # build detection message (JSON in String)
        out = {
            'timestamp': now,
            'frame_id': msg.header.frame_id if hasattr(msg, 'header') else '',
            'detections': detections
        }
        out_msg = String()
        out_msg.data = json.dumps(out)
        self.pub_det.publish(out_msg)

        # publish annotated image if requested
        if self.publish_image:
            try:
                annotated = res.plot() if hasattr(res, 'plot') else frame
                img_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
                self.pub_img.publish(img_msg)
            except Exception as e:
                self.get_logger().warn(f'Failed to publish annotated image: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
