#!/usr/bin/env python3
"""ROS adapter around an externally installed Contact-GraspNet checkout.

The model is intentionally loaded lazily: the ROS workspace remains Humble-only and
the node still starts (and reports a clear warning) when the optional model env is absent.
"""
import importlib
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose
from yolov8_msgs.msg import Yolov8Inference
from grasp_utils import depth_to_point_cloud, select_candidate

class GraspBackend:
    def __init__(self, checkpoint, module_name, class_name, logger):
        self.logger, self.model = logger, None
        if not checkpoint:
            logger.warning('checkpoint_path is empty; grasp inference is unavailable')
            return
        try:
            mod = importlib.import_module(module_name)
            loader = getattr(mod, class_name, None) or getattr(mod, 'ContactGraspNetModel', None)
            if loader is None:
                raise RuntimeError('contact_graspnet.ContactGraspNet was not found')
            try:
                self.model = loader(checkpoint_path=checkpoint)
            except TypeError:
                self.model = loader(checkpoint)
            logger.info(f'Loaded Contact-GraspNet checkpoint: {checkpoint}')
        except Exception as exc:
            logger.error(f'Contact-GraspNet unavailable ({exc}). Node will publish no learned pose.')

    def predict(self, points, colors):
        if self.model is None:
            return []
        # Keep this boundary small: local Contact-GraspNet forks expose different APIs.
        if hasattr(self.model, 'predict_scene'):
            return self.model.predict_scene(points, colors)
        if hasattr(self.model, 'predict'):
            return self.model.predict(points, colors)
        return []

class GraspPoseNode(Node):
    def __init__(self):
        super().__init__('rosie_grasp')
        p = lambda n, v: self.declare_parameter(n, v).value
        self.conf = float(p('confidence_threshold', .5)); self.min_z = float(p('min_depth_m', .15)); self.max_z = float(p('max_depth_m', 2.0))
        self.bounds = list(p('point_cloud_bounds', [-2., 2., -2., 2., .15, 2.]))
        self.crop = bool(p('crop_to_detection', True)); self.target_class = str(p('target_class', ''))
        self.camera_frame = str(p('camera_frame', 'camera_color_optical_frame')); self.base_frame = str(p('base_frame', 'base_link'))
        self.debug = bool(p('debug', True))
        self.bridge = CvBridge(); self.latest_detection = None
        self.tf = Buffer(); self.listener = TransformListener(self.tf, self)
        self.pose_pub = self.create_publisher(PoseStamped, str(p('grasp_topic', '/grasp_pose')), 10)
        self.candidate_pub = self.create_publisher(PoseArray, str(p('candidates_topic', '/grasp_candidates')), 10)
        self.score_pub = self.create_publisher(Float32MultiArray, str(p('candidate_scores_topic', '/grasp_candidate_scores')), 10)
        self.create_subscription(Yolov8Inference, str(p('detections_topic', '/Yolov8_Inference')), self.detection_cb, 10)
        self.rgb = Subscriber(self, Image, str(p('rgb_topic', '/image_raw'))); self.depth = Subscriber(self, Image, str(p('depth_topic', '/depth'))); self.info = Subscriber(self, CameraInfo, str(p('camera_info_topic', '/camera_info')))
        self.sync = ApproximateTimeSynchronizer([self.rgb, self.depth, self.info], 10, .15); self.sync.registerCallback(self.image_cb)
        self.backend = GraspBackend(str(p('checkpoint_path', '')), str(p('backend_module', 'contact_graspnet')), str(p('backend_class', 'ContactGraspNet')), self.get_logger())

    def detection_cb(self, msg):
        choices = [d for d in msg.yolov8_inference if not self.target_class or d.class_name == self.target_class]
        self.latest_detection = choices[0] if choices else None

    def image_cb(self, rgb_msg, depth_msg, info):
        try:
            rgb = self.bridge.imgmsg_to_cv2(rgb_msg, 'rgb8'); depth = self.bridge.imgmsg_to_cv2(depth_msg, 'passthrough').astype(np.float32)
            if depth_msg.encoding in ('16UC1', 'mono16'): depth *= .001
            h, w = depth.shape[:2]; valid = np.ones((h, w), dtype=bool)
            if self.crop and self.latest_detection is not None and len(self.latest_detection.coordinates) >= 8:
                c = np.asarray(self.latest_detection.coordinates, dtype=np.float32).reshape(-1, 2); x0,y0 = np.floor(c.min(0)).astype(int); x1,y1 = np.ceil(c.max(0)).astype(int); valid[:,:] &= False; valid[max(0,y0):min(h,y1+1), max(0,x0):min(w,x1+1)] = True
            points, valid = depth_to_point_cloud(depth, info.k, self.min_z, self.max_z, self.bounds, valid)
            colors = rgb[valid]
            if len(points) < 32: return
            candidates = self.backend.predict(points, colors)
            self.publish_candidates(candidates, rgb_msg.header.stamp)
            selected = select_candidate(candidates, self.conf)
            if selected is None: return
            self.get_logger().info(f"Selected learned grasp score={selected['score']:.3f} frame={self.camera_frame}")
            pose = PoseStamped(); pose.header = rgb_msg.header; pose.header.frame_id = self.camera_frame
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = map(float, selected['position']); pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w = map(float, selected['quaternion'])
            transform = self.tf.lookup_transform(self.base_frame, pose.header.frame_id, rclpy.time.Time())
            out = do_transform_pose(pose, transform); out.header.stamp = rgb_msg.header.stamp; self.pose_pub.publish(out)
        except (TransformException, ValueError, RuntimeError) as exc:
            self.get_logger().warning(f'Grasp inference skipped: {exc}')

    def normalize(self, c):
        if isinstance(c, dict): return c
        if hasattr(c, 'position') and hasattr(c, 'quaternion'): return {'position': c.position, 'quaternion': c.quaternion, 'score': getattr(c, 'score', 0.)}
        return None

    def publish_candidates(self, candidates, stamp):
        if not self.debug: return
        msg = PoseArray(); msg.header.frame_id = self.camera_frame; msg.header.stamp = stamp
        scores = Float32MultiArray()
        for raw in candidates or []:
            c = self.normalize(raw)
            if not c: continue
            p = np.asarray(c.get('position', []), float); q = np.asarray(c.get('quaternion', []), float)
            if p.shape == (3,) and q.shape == (4,):
                from geometry_msgs.msg import Pose
                pose = Pose(); pose.position.x,pose.position.y,pose.position.z = map(float,p); pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w = map(float,q); msg.poses.append(pose); scores.data.append(float(c.get('score', c.get('confidence', 0.0))))
        self.candidate_pub.publish(msg)
        self.score_pub.publish(scores)

def main(args=None):
    rclpy.init(args=args); node = GraspPoseNode(); rclpy.spin(node); node.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
