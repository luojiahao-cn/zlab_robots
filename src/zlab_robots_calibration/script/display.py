#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import image_geometry

# 订阅一个摄像头的输入图像
# 订阅这个摄像头的内参数
# 订阅一个camera_frame
# 订阅若干个要投射到图像上的target_frame，用列表存储，可以指定绘制的类型，三轴(axis)，单轴(z_axis)或者单点(point)
# 用tf工具查询坐标变换，将target_frame绘制在图像上

class FrameReprojector:
    def __init__(self):
        rospy.init_node('frame_reprojector_node', anonymous=True)
        
        # 参数获取
        self.image_topic = rospy.get_param('~image_topic', '/zed2i/zed_node/left/image_rect_gray')
        self.camera_info_topic = rospy.get_param('~camera_info_topic', '/zed2i/zed_node/left/camera_info')
        # 默认使用相机的光学frame，如果传了参数则使用自定义的camera_frame
        self.camera_frame = rospy.get_param('~camera_frame', 'rig') 
        
        # target_frames 格式约定: [{'frame': 'target1', 'type': 'axes', 'length': 0.1}, 
        #                         {'frame': 'target2', 'type': 'point'},
        #                         {'frame': 'target3', 'type': 'z_axis', 'length': 0.1}]
        self.target_frames = rospy.get_param('~target_frames', [
            {'frame': 'diana7_em_tcp_filt', 'type': 'axes', 'length': 0.1},
            {'frame': 'arm1_em_tcp_filt', 'type': 'axes', 'length': 0.1},
            {'frame': 'arm2_em_tcp_filt', 'type': 'axes', 'length': 0.1},
            {'frame': 'sensor_array_filt', 'type': 'point'}
        ])
        
        self.show_window = rospy.get_param('~show_window', False)

        self.bridge = CvBridge()
        self.cam_model = image_geometry.PinholeCameraModel()
        self.has_cam_info = False

        # TF2 初始化
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 订阅与发布
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cam_info_cb)
        rospy.Subscriber(self.image_topic, Image, self.image_cb, queue_size=1)
        self.image_pub = rospy.Publisher('~reprojected_image', Image, queue_size=1)

        rospy.loginfo("FrameReprojector initialized. Waiting for image and camera info...")

    def cam_info_cb(self, msg):
        if not self.has_cam_info:
            self.cam_model.fromCameraInfo(msg)
            self.has_cam_info = True
            if not self.camera_frame:
                self.camera_frame = msg.header.frame_id
            rospy.loginfo(f"Received CameraInfo. Using camera_frame: {self.camera_frame}")

    def transform_point(self, target_frame, source_frame, xyz):
        """将源坐标系的某一点转换到目标坐标系下"""
        pt_stamped = PointStamped()
        pt_stamped.header.frame_id = source_frame
        pt_stamped.header.stamp = rospy.Time(0) # 获取最新可用的变换
        pt_stamped.point.x, pt_stamped.point.y, pt_stamped.point.z = xyz

        try:
            # target_frame 通常是 camera_frame
            transformed_pt = self.tf_buffer.transform(pt_stamped, target_frame, rospy.Duration(0.1))
            return transformed_pt.point
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            return None

    def draw_axes(self, cv_image, frame_id, length=0.1):
        """绘制三轴 (RGB = XYZ)"""
        origin = self.transform_point(self.camera_frame, frame_id, (0, 0, 0))
        pt_x = self.transform_point(self.camera_frame, frame_id, (length, 0, 0))
        pt_y = self.transform_point(self.camera_frame, frame_id, (0, length, 0))
        pt_z = self.transform_point(self.camera_frame, frame_id, (0, 0, length))

        if not all([origin, pt_x, pt_y, pt_z]):
            return

        # 要求Z必须在相机前方才能投影 (简单的 z > 0 判断)
        if origin.z <= 0: return

        uv_origin = tuple(map(int, self.cam_model.project3dToPixel((origin.x, origin.y, origin.z))))
        uv_x = tuple(map(int, self.cam_model.project3dToPixel((pt_x.x, pt_x.y, pt_x.z))))
        uv_y = tuple(map(int, self.cam_model.project3dToPixel((pt_y.x, pt_y.y, pt_y.z))))
        uv_z = tuple(map(int, self.cam_model.project3dToPixel((pt_z.x, pt_z.y, pt_z.z))))

        thickness = 2
        cv2.line(cv_image, uv_origin, uv_x, (0, 0, 255), thickness) # X轴红色
        cv2.line(cv_image, uv_origin, uv_y, (0, 255, 0), thickness) # Y轴绿色
        cv2.line(cv_image, uv_origin, uv_z, (255, 0, 0), thickness) # Z轴蓝色
        cv2.putText(cv_image, frame_id, uv_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

    def draw_z_axis(self, cv_image, frame_id, length=0.1):
        """只绘制单轴 (Z轴为例)"""
        origin = self.transform_point(self.camera_frame, frame_id, (0, 0, 0))
        pt_z = self.transform_point(self.camera_frame, frame_id, (0, 0, length))

        if not all([origin, pt_z]) or origin.z <= 0:
            return

        uv_origin = tuple(map(int, self.cam_model.project3dToPixel((origin.x, origin.y, origin.z))))
        uv_z = tuple(map(int, self.cam_model.project3dToPixel((pt_z.x, pt_z.y, pt_z.z))))

        cv2.line(cv_image, uv_origin, uv_z, (255, 0, 0), 2)
        cv2.putText(cv_image, f"{frame_id}_Z", uv_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)

    def draw_point(self, cv_image, frame_id):
        """只绘制单点 (原点)"""
        origin = self.transform_point(self.camera_frame, frame_id, (0, 0, 0))

        if not origin or origin.z <= 0:
            return

        uv_origin = tuple(map(int, self.cam_model.project3dToPixel((origin.x, origin.y, origin.z))))
        cv2.circle(cv_image, uv_origin, 5, (0, 255, 255), -1)
        cv2.putText(cv_image, frame_id, uv_origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    def image_cb(self, msg):
        if not self.has_cam_info or not self.camera_frame:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        # 遍历要投射的坐标系配置进行渲染
        for target in self.target_frames:
            frame_id = target.get('frame')
            draw_type = target.get('type', 'axes')
            length = target.get('length', 0.1)

            if not frame_id:
                continue

            if draw_type == 'axes':
                self.draw_axes(cv_image, frame_id, length)
            elif draw_type == 'z_axis':
                self.draw_z_axis(cv_image, frame_id, length)
            elif draw_type == 'point':
                self.draw_point(cv_image, frame_id)

        # 发布画好的图像
        try:
            out_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            out_msg.header = msg.header
            self.image_pub.publish(out_msg)
            if self.show_window:
                cv2.imshow("Reprojected Image", cv_image)
                cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        node = FrameReprojector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

