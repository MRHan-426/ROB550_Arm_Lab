#!/usr/bin/env python3

"""!
Class to represent the camera.
"""
 
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor, MultiThreadedExecutor

import cv2
import time
from datetime import datetime

import numpy as np
from PyQt5.QtGui import QImage
from PyQt5.QtCore import QThread, pyqtSignal, QTimer
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CameraInfo
from apriltag_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from block_detection import *

# default
INTRINISC_MATRIX = np.array([[918.2490195188435, 0.0, 636.4533753942957],
                            [0.0, 912.0611927215057, 365.23840749139805],
                            [0.0, 0.0, 1.0]])

DISTORTION = np.array([0.12255661041263047, -0.19338918906656302, 0.00411197288757392, 0.007337075149104217, 0.0])


class Camera():
    """!
    @brief      This class describes a camera.
    """

    def __init__(self):
        """!
        @brief      Construcfalsets a new instance.
        """
        self.VideoFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.GridFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((720,1280)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((720,1280, 3)).astype(np.uint8)
        self.DepthFrameRGB = np.zeros((720,1280, 3)).astype(np.uint8)
        self.tagsCenter = []
        self.tagsNotChange = []
        self.boundary = [
            # Whole workspace
            (self.inv_affline_transformation([[200, 75],[200, 700],[1100, 700],[1100, 75]]), 1),
            (self.inv_affline_transformation([[570, 430],[570, 700],[735, 700],[735, 430]]), 0),
            # Positive Half
            (self.inv_affline_transformation([[200, 75],[200, 540],[1100, 540],[1100, 75]]), 1),
            (self.inv_affline_transformation([[570, 365],[570, 545],[735, 545],[735, 365]]), 0),
            # Task Three
            (self.inv_affline_transformation([[200, 75],[200, 448],[1100, 448],[1100, 75]]), 1),
            (self.inv_affline_transformation([[570, 365],[570, 448],[735, 448],[735, 365]]), 0),
            (self.inv_affline_transformation([[200, 450],[200, 700],[1100, 700],[1100, 450]]), 1),
            (self.inv_affline_transformation([[570, 450],[570, 700],[735, 700],[735, 450]]), 0),
            # Work space for task3 [8:10]
            (self.inv_affline_transformation([[400, 150],[400, 350],[900, 350],[900, 150]]), 1),
            (self.inv_affline_transformation([[570, 450],[570, 700],[735, 700],[735, 450]]), 0),
            # task3 except work space [10:13]
            (self.inv_affline_transformation([[200, 75],[200, 700],[1100, 700],[1100, 75]]), 1),
            (self.inv_affline_transformation([[570, 365],[570, 700],[735, 700],[735, 365]]), 0),
            (self.inv_affline_transformation([[400, 150],[400, 350],[900, 350],[900, 150]]), 0)
        ]
        # print("===========================================")
        # print("boundary list:")
        # print(self.boundary)
        # print("===========================================")
        self.transformation_matrix = np.zeros((3,3))

        # mouse clicks & calibration variables
        self.cameraCalibrated = False
        self.intrinsic_matrix = INTRINISC_MATRIX
        self.distortion = DISTORTION
        self.extrinsic_matrix = np.loadtxt("../config/extrinsic.txt", delimiter=', ')
        self.use_default_intrinisc_matrix = False
        self.last_click = np.array([0, 0])
        self.last_click_worldframe = (0.0, 0.0, 0.0)
        self.new_click = False
        self.rgb_click_points = np.zeros((5, 2), int)
        self.depth_click_points = np.zeros((5, 2), int)
        self.grid_x_points = np.arange(-500, 501, 50)
        self.grid_y_points = np.arange(-175, 525, 50)
        self.grid_points = np.array(np.meshgrid(self.grid_x_points, self.grid_y_points))
        self.tag_detections = np.array([])
        self.tag_locations_3D = [[-250, -25, 0],[250, -25, 0],[250, 275, 0],[-250, 275, 0], [-475,-100,150], [475,400,150]]
        self.tag_locations_2D = [[400, 550], [900, 550], [900, 250], [400, 250]]

        """ block info """
        self.detect_blocks = False
        self.blocks = []
        self.block_contours = np.array([])
        self.block_detections = np.array([])

    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_contours, -1,
                         (255, 0, 255), 3)

    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[..., 0] = self.DepthFrameRaw >> 1
        self.DepthFrameHSV[..., 1] = 0xFF
        self.DepthFrameHSV[..., 2] = 0x9F
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,
                                          cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png", cv2.IMREAD_UNCHANGED),
            cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",
                                        0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtGridFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.GridFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtDepthFrame(self):
        """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
        try:
            img = QImage(self.DepthFrameRGB, self.DepthFrameRGB.shape[1],
                         self.DepthFrameRGB.shape[0], QImage.Format_RGB888)
            return img
        except:
            return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (1280, 720))
            img = QImage(frame, frame.shape[1], frame.shape[0],
                         QImage.Format_RGB888)
            return img
        except:
            return None

    def inv_affline_transformation(self, points: list) -> list:
        """
        @brief Perform the inverse affine transformation on a list of points.

        This method applies the inverse of the affine transformation matrix to a list
        of points, converting them from transformed coordinates to their original coordinates.

        @param points: A list of points, where each point is represented as [x, y].

        @return list: A list of original points after the inverse transformation.
        """
        inv_transformation_matrix = np.linalg.inv(self.transformation_matrix)
        original_points = []
        for point in points:
            transformed_coords = np.array([point[0], point[1], 1])
            original_coords = np.matmul(inv_transformation_matrix, transformed_coords)
            original_x = original_coords[0] / original_coords[2]
            original_y = original_coords[1] / original_coords[2]
            original_points.append([int(original_x), int(original_y)])
        return np.array(original_points)

    def depth_correction(self, depth_img):
        """
        @brief Perform depth correction on a depth image.

        This method corrects the depth values in a depth image to obtain 3D world coordinates
        for each pixel.

        @param depth_img: A 2D numpy array representing the depth image.

        @return numpy.ndarray: A 3D numpy array representing the corrected 3D world coordinates
                            for each pixel.
        """
        y = np.arange(depth_img.shape[0])
        x = np.arange(depth_img.shape[1])
        mesh_x, mesh_y = np.meshgrid(x, y)
        Z = depth_img.astype(np.float32)
        X = (mesh_x - self.intrinsic_matrix[0, 2]) * Z / self.intrinsic_matrix[0, 0]
        Y = (mesh_y - self.intrinsic_matrix[1, 2]) * Z / self.intrinsic_matrix[1, 1]

        homogeneous_coordinates = np.stack((X, Y, Z, np.ones_like(Z)), axis=-1)
        P_c = homogeneous_coordinates.reshape(-1, 4).T
        P_w = np.linalg.inv(self.extrinsic_matrix) @ P_c
        points_3d = P_w.T[:, 2].reshape(depth_img.shape[0], depth_img.shape[1], 1)

        return points_3d

    def projectGridInRGBImage(self):
        """!
        @brief      projects (when we are not doing check point2, we do not use this function)

                    Use the intrinsic and extrinsic matricies to project the gridpoints 
                    on the board into pixel coordinates. copy self.VideoFrame to self.GridFrame and
                    and draw on self.GridFrame the grid intersection points from self.grid_points
                    (hint: use the cv2.circle function to draw circles on the image)
        """
        if self.cameraCalibrated == True:
            modified_image = self.VideoFrame.copy()
            # # Draw Grid of Points
            X, Y = np.meshgrid(self.grid_x_points, self.grid_y_points)
            grid_points = np.vstack((X.ravel(), Y.ravel(), (np.ones_like(X).ravel()), np.ones_like(X).ravel())).astype(np.float32)
            camera_points = np.dot(self.extrinsic_matrix[0:3, :], grid_points)
            pixel_points = np.dot(self.intrinsic_matrix, camera_points)
            pixel_x = pixel_points[0, :] / pixel_points[2, :]
            pixel_y = pixel_points[1, :] / pixel_points[2, :]
            for px, py in zip(pixel_x, pixel_y):
                point_pos = (int(px), int(py))
                cv2.circle(modified_image, point_pos, 3, (0, 255, 0), thickness=-1)
            self.VideoFrame = modified_image

    def birdEyesViewInGridFrame(self):
        """
        @brief Perform bird's-eye view transformation on the RGB image.

        This method computes the transformation matrix M for bird's-eye view transformation
        using image points and corresponding world points. It then applies the transformation
        to the RGB image to obtain a bird's-eye view image in the grid frame.

        @return None
        """
        rgb_img = self.VideoFrame.copy()
        image_points = np.array(self.tagsNotChange[:4]).astype(np.float32)
        world_points = np.array(self.tag_locations_2D).astype(np.float32)

        # Compute transformation matrix M
        if image_points.shape[0] <= 3 or world_points.shape[0] <= 3:
            print("Wait for video input")
            return None
        elif image_points.shape[0] != world_points.shape[0]:
            print("ERROR: Image points do not match Camera points")
            print(image_points.shape)
            return None
        elif image_points.shape[0] >= 4 and world_points.shape[0] >= 4:
            self.transformation_matrix = cv2.getPerspectiveTransform(image_points, world_points)
            if self.cameraCalibrated == True:
                modified_image = cv2.warpPerspective(rgb_img, self.transformation_matrix, (rgb_img.shape[1], rgb_img.shape[0]))
                self.GridFrame = modified_image

    def drawTagsInRGBImage(self, msg):
        """
        @brief      Draw tags from the tag detection

                    Use the tag detections output, to draw the corners/center/tagID of
                    the apriltags on the copy of the RGB image. And output the video to self.TagImageFrame.
                    Message type can be found here: /opt/ros/humble/share/apriltag_msgs/msg

                    center of the tag: (detection.centre.x, detection.centre.y) they are floats
                    id of the tag: detection.id
        """
        
        modified_image = self.VideoFrame.copy()
        self.tagsCenter = []
     
        for detection in msg.detections:
            # Draw center
            center = [int(detection.centre.x), int(detection.centre.y)]
            self.tagsCenter.append(center)
            cv2.circle(modified_image, center, 3, (0, 255, 0), thickness=-1)
            # Draw edges
            corner1 = [int(detection.corners[0].x),int(detection.corners[0].y)]
            corner2 = [int(detection.corners[1].x),int(detection.corners[1].y)]
            corner3 = [int(detection.corners[2].x),int(detection.corners[2].y)]
            corner4 = [int(detection.corners[3].x),int(detection.corners[3].y)]
            cv2.line(modified_image, corner1,corner2, (0, 0, 255), thickness=2)
            cv2.line(modified_image, corner2,corner3, (0, 0, 255), thickness=2)
            cv2.line(modified_image, corner3,corner4, (0, 0, 255), thickness=2)
            cv2.line(modified_image, corner4,corner1, (0, 0, 255), thickness=2)
            # Draw ID
            ID_pos = [center[0]+20,center[1]-20]
            ID = "ID: " + str(detection.id)
            #print(ID)
            cv2.putText(modified_image,ID,ID_pos,cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),thickness=2)
        
        if self.tagsNotChange == []:
            self.tagsNotChange = self.tagsCenter
        
        self.TagImageFrame = modified_image


    def calibrateFromAprilTag(self, msg):
        self.tagsNotChange = []
        image_points = np.array(self.tagsCenter).astype(np.float32)
        world_points = np.array(self.tag_locations_3D).astype(np.float32)

        if image_points.shape[0] <= 3 or world_points.shape[0] <= 3:
            print("ERROR: AprilTag not enough, at least four")
            return None
        elif image_points.shape[0] != world_points.shape[0]:
            print("ERROR: Image points do not match Camera points")
            return None
        elif image_points.shape[0] >= 4 and world_points.shape[0] >= 4:
            success, rvec, T = cv2.solvePnP(world_points, image_points, self.intrinsic_matrix, self.distortion)
        
        R, _ = cv2.Rodrigues(rvec)
        self.extrinsic_matrix = np.concatenate((R, T), 1)
        self.extrinsic_matrix = np.vstack((self.extrinsic_matrix, [0,0,0,1]))
        print("Calibrition done")
        print("======================================================")
        print("Extrinsic Matrix:")
        print(self.extrinsic_matrix)
        print("======================================================")
        self.cameraCalibrated = True

        filename = "../config/extrinsic.txt"
        np.savetxt(filename, self.extrinsic_matrix, fmt='%10f', delimiter=', ')

            
    def transformFromImageToWorldFrame(self, pos: tuple) -> list:
        """!
        @brief      transform From Image To WorldFrame

        @param      pos: x, y z coordinate in image frame
                    pos[0]:x, pos[1]:y
        """
        
        z = self.DepthFrameRaw[int(pos[1])][int(pos[0])]
        if self.DepthFrameRaw.any() != 0:
            
            if self.cameraCalibrated == True:
                T = self.extrinsic_matrix[:,3]
                R = self.extrinsic_matrix[:,0:3]

                K2 = self.extrinsic_matrix
                K1 = self.intrinsic_matrix

                P_w =  np.linalg.inv(K2) @ np.vstack((z * np.linalg.inv(K1) @ np.array([[pos[0]],[pos[1]],[1]]), 1))
            
            else: # use default extrinsic parameters
                t = np.array([0, 365, 1000])
                R = np.array([[0.999, 0.0094, -0.0426],
                            [0.0, -0.9763,  -0.2164],
                            [-0.0436, 0.2162, -0.9754]])

                u = pos[0]
                v = pos[1]
                fx, fy = self.intrinsic_matrix[0, 0], self.intrinsic_matrix[1, 1]
                cx, cy = self.intrinsic_matrix[0, 2], self.intrinsic_matrix[1, 2]
                x_prime = (u - cx) / fx
                y_prime = (v - cy) / fy
                X_c = z * x_prime
                Y_c = z * y_prime
                Z_c = z
                P_c = np.array([X_c, Y_c, Z_c]).reshape(3, 1)
                t = t.reshape(3,1)
                P_w = R @ P_c + t
            return [P_w[0][0], P_w[1][0], P_w[2][0]]
        else:
            print("ERROR: Cannot load depth information")
            return [0, 0, 0]


class ImageListener(Node):
    def __init__(self, topic, camera):
        super().__init__('image_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
        except CvBridgeError as e:
            print(e)
        self.camera.VideoFrame = cv_image


class TagDetectionListener(Node):
    def __init__(self, topic, camera):
        super().__init__('tag_detection_listener')
        self.topic = topic
        self.tag_sub = self.create_subscription(
            AprilTagDetectionArray,
            topic,
            self.callback,
            10
        )
        self.camera = camera

    def callback(self, msg):
        self.camera.tag_detections = msg
        if np.any(self.camera.VideoFrame != 0):
            self.camera.drawTagsInRGBImage(msg)


class JBCalibrateListener(Node):
    def __init__(self, topic, camera):
        super().__init__('JB_calibrate_listener')
        self.topic = topic
        self.calibrate_sub = self.create_subscription(
            Int32,
            topic,
            self.callback,
            10
        )
        self.camera = camera

    def callback(self, msg):
        if msg.data == 1:
            self.camera.cameraCalibrated = False
            self.camera.calibrateFromAprilTag(msg)


class JBDetectListener(Node):
    def __init__(self, topic, camera):
        super().__init__('JB_Detect_listener')
        self.topic = topic
        self.detect_sub = self.create_subscription(
            Int32,
            topic,
            self.callback,
            10
        )
        self.camera = camera

    def callback(self, msg):
        if msg.data == 1:
            self.camera.detect_blocks = True
        else:
            self.camera.detect_blocks = False



class CameraInfoListener(Node):
    def __init__(self, topic, camera):
        super().__init__('camera_info_listener')  
        self.topic = topic
        self.tag_sub = self.create_subscription(CameraInfo, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        if self.camera.use_default_intrinisc_matrix == True:
            self.camera.intrinsic_matrix = np.reshape(data.k, (3, 3))
            self.camera.distortion = np.array(data.d)
        else:
            self.camera.intrinsic_matrix = INTRINISC_MATRIX


class DepthListener(Node):
    def __init__(self, topic, camera):
        super().__init__('depth_listener')
        self.topic = topic
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, topic, self.callback, 10)
        self.camera = camera

    def callback(self, data):
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
            # cv_depth = cv2.rotate(cv_depth, cv2.ROTATE_180)
        except CvBridgeError as e:
            print(e)
        self.camera.DepthFrameRaw = cv_depth
        # self.camera.DepthFrameRaw = self.camera.DepthFrameRaw / 2
        self.camera.ColorizeDepthFrame()


class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent)
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_detection_topic = "/detections"
        JB_calibrate_topic = "/JB_calibrate"
        JB_Detect_topic = "/JB_Detect"

        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic,
                                                  self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic,
                                                      self.camera)
        JB_calibrate_listener = JBCalibrateListener(JB_calibrate_topic,
                                                      self.camera)  
        JB_Detect_listener = JBDetectListener(JB_Detect_topic,
                                                      self.camera)  

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(image_listener)
        self.executor.add_node(depth_listener)
        self.executor.add_node(camera_info_listener)
        self.executor.add_node(tag_detection_listener)
        self.executor.add_node(JB_calibrate_listener)
        self.executor.add_node(JB_Detect_listener)


    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Grid window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        try:
            while rclpy.ok():
                start_time = time.time()
                rgb_frame = self.camera.convertQtVideoFrame()
                depth_frame = self.camera.convertQtDepthFrame()
                tag_frame = self.camera.convertQtTagImageFrame()
                # this function is for check point2
                # self.camera.projectGridInRGBImage()
                self.camera.birdEyesViewInGridFrame()
                grid_frame = self.camera.convertQtGridFrame()
                if ((rgb_frame != None) & (depth_frame != None)):
                    self.updateFrame.emit(
                        rgb_frame, depth_frame, tag_frame, grid_frame)
                self.executor.spin_once() # comment this out when run this file alone.
                elapsed_time = time.time() - start_time
                sleep_time = max(0.03 - elapsed_time, 0)
                time.sleep(sleep_time)

                if __name__ == '__main__':
                    cv2.imshow(
                        "Image window",
                        cv2.cvtColor(self.camera.VideoFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                    cv2.imshow(
                        "Tag window",
                        cv2.cvtColor(self.camera.TagImageFrame, cv2.COLOR_RGB2BGR))
                    cv2.imshow("Grid window",
                        cv2.cvtColor(self.camera.GridFrame, cv2.COLOR_RGB2BGR))
                    cv2.waitKey(3)
                    time.sleep(0.03)
        except KeyboardInterrupt:
            pass
        
        self.executor.shutdown()
        

def main(args=None):
    rclpy.init(args=args)
    try:
        camera = Camera()
        videoThread = VideoThread(camera)
        videoThread.start()
        try:
            videoThread.executor.spin()
        finally:
            videoThread.executor.shutdown()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()