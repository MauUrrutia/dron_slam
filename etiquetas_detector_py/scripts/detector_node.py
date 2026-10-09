#!/usr/bin/env python3
import json
import os
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from etiquetas_detector_py.detector import Detector


class DetectorNode(Node):
    def __init__(self):
        super().__init__('etiquetas_detector')
        self.declare_parameter('model_path', '')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('conf', 0.5)
        self.declare_parameter('iou', 0.45)
        self.declare_parameter('roi_ext', 50)
        self.declare_parameter('publish_debug_image', True)
        self.declare_parameter('frame_skip', 3)

        requested = self.get_parameter('model_path').value
        model_path = requested
        if not model_path or not os.path.exists(model_path):
            self.get_logger().warn(f"No existe '{requested}', probando ruta del paquete")
            model_path = os.path.join(
                get_package_share_directory('etiquetas_detector_py'),
                'models', 'best_Quala.pt')
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Modelo no encontrado: {model_path}")

        self.detector = Detector(model_path)
        self.bridge = CvBridge()
        self.pub_det = self.create_publisher(String, 'detecciones', 10)
        self.pub_img = self.create_publisher(Image, 'detecciones/imagen', 1)

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(
            Image, self.get_parameter('image_topic').value, self.callback, qos)

        self._lock = threading.Lock()
        self._event = threading.Event()
        self._pending = None
        self._running = True
        self._count = 0
        self._worker = threading.Thread(target=self._loop, daemon=True)
        self._worker.start()
        self.get_logger().info('Detector listo')

    def callback(self, msg):
        self._count += 1
        if self._count % max(1, self.get_parameter('frame_skip').value) != 0:
            return
        with self._lock:
            self._pending = msg
        self._event.set()

    def _loop(self):
        while self._running:
            self._event.wait(timeout=1.0)
            self._event.clear()
            with self._lock:
                msg, self._pending = self._pending, None
            if msg is None:
                continue
            try:
                self._process(msg)
            except Exception as e:
                self.get_logger().error(f"Error: {e}")

    def _process(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        draw = self.get_parameter('publish_debug_image').value
        datos = self.detector.detect(
            frame,
            conf=self.get_parameter('conf').value,
            iou=self.get_parameter('iou').value,
            roi_extension=self.get_parameter('roi_ext').value,
            draw=draw)
        if datos['objetos']:
            self.pub_det.publish(String(data=json.dumps(datos)))
        if draw:
            dbg = self.detector.get_debug_image()
            if dbg is not None:
                out = self.bridge.cv2_to_imgmsg(dbg, encoding='bgr8')
                out.header = msg.header
                self.pub_img.publish(out)

    def destroy_node(self):
        self._running = False
        self._event.set()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()