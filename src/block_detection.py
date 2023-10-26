import cv2
import numpy as np
from sklearn.cluster import KMeans
from collections import Counter
from dataclasses import dataclass
from scipy import stats
import pickle
import warnings
warnings.filterwarnings("ignore", message="Trying to unpickle estimator SVC from version", category=UserWarning)


font = cv2.FONT_HERSHEY_SIMPLEX


class block:
    def __init__(self, center, depth, orientation, contour, color, type, stack = False):
        self.center = center # height, width
        self.depth = depth
        self.orientation = np.deg2rad(orientation)
        self.contour = contour
        self.color = color
        self.type = type
        self.stack = stack

    # detect whether a point is in a square
    def inArea(self, point):
        width, height = point
        print("===========================================")
        print("point:", (height, self.center[0]))
        print("center:", (width, self.center[1]))
        print("===========================================")

        if np.square(height - self.center[0]) + np.square(width - self.center[1]) < 20 * 20:
            return True
        return False


class detection:
    def __init__(self, camera, boundary, lower = 850, upper = 1100):
        self.blocks = []
        self.camera = camera
        self.lower = lower
        self.upper = upper
        self.boundary = [(np.array([[ 200, 75],[ 1100,  75],[1100,  700],[200,  700]]), 1),
                        (np.array([[ 565, 437],[ 735,  437],[735,  700],[565,  700]]), 0)] # whole workspace
        self.rgb_img = self.camera.VideoFrame.copy()
        self.depth_raw = self.camera.DepthFrameRaw.copy()
        self.only_blocks = True

    def reset(self):
        self.blocks = []
        self.rgb_img = self.camera.VideoFrame.copy()
        self.depth_raw = self.camera.DepthFrameRaw.copy()

    def run(self, only_blocks = True, boundary = None):
        self.rgb_img = self.camera.VideoFrame.copy()
        self.depth_raw = self.camera.DepthFrameRaw.copy()
        self.only_blocks = only_blocks
        if boundary != None:
            self.boundary = boundary

        self._detectBlocksUsingCluster()


    def _detectBlocksUsingCluster(self):
        """!
        @brief      Detect blocks and refine their contour using cluster.
                    1. Filter by height to eliminate the robot arm.
                    2. Filter the grid through hsv frame.
                    3. Find contours and create a subregion for each contour.
                    4. Perform depth-based detection to determine if it forms a stack.
                    5. Utilize clustering for refining each subregion.
                    6. Find contours again, return precise position and block type.

        @param      rgb_img: image from Video Frame, in form of RGB.
                    depth_raw: depth image from DepthFrameRaw.
                    boundary: boundary of workspace.
                    only_blocks: Only detect blocks.
                    lower & upper: Height restriction.
        
        @return     blocks: List of block class.
        """
        # 1. Filter by height to eliminate the robot arm.
        height_filter = cv2.inRange(self.depth_raw, self.lower, self.upper)
        mask = cv2.merge([height_filter, height_filter, height_filter])
        rgb_img_remove_robot_arm = cv2.bitwise_and(self.rgb_img, mask)

        # 2. Filter the grid through hsv frame.
        hsv_img = cv2.cvtColor(rgb_img_remove_robot_arm, cv2.COLOR_RGB2HSV)
        hsv_mask = cv2.inRange(hsv_img, np.array([0,43,46]), np.array([180, 255, 255]))
        # hsv_mask = cv2.medianBlur(hsv_mask, 3)
        boundary_mask = np.zeros_like(hsv_mask, dtype=np.uint8)
        for item in self.boundary:
            if item[1] == 1:
                cv2.fillPoly(boundary_mask, [item[0]], 255)
            else:
                cv2.fillPoly(boundary_mask, [item[0]], 0)
        masked_hsv = cv2.bitwise_and(hsv_mask, boundary_mask)

        # 3. Find contours and create a subregion for each contour.
        raw_contours, _ = cv2.findContours(masked_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
        for raw_contour in raw_contours:
            M = cv2.moments(raw_contour)
            if M['m00'] < 200 or abs(M["m00"]) > 7000:
                continue
            if M["m00"] < 850:
                sub_region_side = 50
            else:
                sub_region_side = 80

            # choose eight points around center to minimize depth error
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            sample_points = [(0, 0), (8, 8), (8, -8), (-8, 8), (-8, -8), (4, 4), (4, -4), (-4, 4), (-4, -4)]
            cz_values = [self.depth_raw[cy + dy, cx + dx] for dy, dx in sample_points]
            cz = min(cz_values)

            # choose subregion
            y_min = max(0, int(cy - sub_region_side / 2))
            y_max = min(720, int(cy + sub_region_side / 2))
            x_min = max(0, int(cx - sub_region_side / 2))
            x_max = min(1280, int(cx + sub_region_side / 2))
            canvas = np.zeros_like(masked_hsv)
            cv2.drawContours(canvas, [raw_contour], 0, 255, -1)
            binary_contour = cv2.bitwise_and(self.rgb_img, self.rgb_img, mask=canvas)
            sub_region = binary_contour[y_min:y_max, x_min:x_max]
            sub_region_depth = self.depth_raw[y_min:y_max, x_min:x_max]

            check_point_depth_mean = (self.depth_raw[y_min,x_min] + self.depth_raw[y_max, x_min] + depth_raw[y_max, x_max] + depth_raw[y_min, x_max])/4
            depth_gap = abs(cz - check_point_depth_mean)

            # 4. Perform depth-based detection to determine if it forms a stack.
            if depth_gap > 45:
                # This is a stack
                stack = True
                # filter the lower level
                height_filter = cv2.inRange(sub_region_depth, int(cz) - 20, int(cz) + 10)
                mask = cv2.merge([height_filter, height_filter, height_filter])
                sub_region = cv2.bitwise_and(sub_region, mask)
                n_colors = 3

            elif depth_gap < 5:
                continue

            elif depth_gap < 30 and self.only_blocks == False:
                # detect semi-circle through depth segementation
                stack = False
                height_filter_1 = cv2.inRange(sub_region, int(cz) - 20, int(cz) + 7)
                height_filter_2 = cv2.inRange(sub_region, int(cz) - 20, int(cz) + 1)
                area_1 = cv2.countNonZero(height_filter_1)
                area_2 = cv2.countNonZero(height_filter_2)
                area_ratio = area_1 / area_2
                if area_ratio > 2.5:
                    type = "semi-circle"
                    epsilon = 0.08 * cv2.arcLength(raw_contour, True)
                    approx = cv2.approxPolyDP(raw_contour, epsilon, True)
                    _, _, orientation = cv2.minAreaRect(approx)
                    color = self._detect_color(raw_contour)
                    a_block = block(center=[cy, cx], 
                                    depth=depth_gap, 
                                    orientation=orientation, 
                                    contour=raw_contour, 
                                    color=color, 
                                    type=type, 
                                    stack=stack)
                    continue
            else:
                stack = False
                n_colors = 4

            # 5. Utilize clustering for refining each subregion.
            success, refined_region, color = self._refine_contour(sub_region, sub_region_depth, n_colors)
            
            if success == False:
                continue
            
            # 6. Find contours again, return precise position and block type.
            precise_contours, _ = cv2.findContours(refined_region, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for precise_contour in precise_contours:
                M = cv2.moments(precise_contour)
                if M['m00'] < 200 or abs(M["m00"]) > 7000:
                    continue
                
                epsilon = 0.08 * cv2.arcLength(precise_contour, True)
                approx = cv2.approxPolyDP(precise_contour, epsilon, True)
                _, (box_width, box_height), box_angle = cv2.minAreaRect(approx)

                if box_width < box_height:
                    box_width, box_height = box_height, box_width
                if box_height < 10:
                    continue
                aspect_ratio = box_width / box_height

                # translation back to the image coordinate
                precise_cx = int(M['m10']/M['m00']) + cx - int(sub_region_side/2)
                precise_cy = int(M['m01']/M['m00']) + cy - int(sub_region_side/2)
                precise_cz = self.depth_raw(precise_cy, precise_cx)
                y_min = int(precise_cy - sub_region_side / 2)
                y_max = int(precise_cy + sub_region_side / 2)
                x_min = int(precise_cx - sub_region_side / 2)
                x_max = int(precise_cx + sub_region_side / 2)
                check_point_depth_mean = (self.depth_raw[y_min,x_min] + self.depth_raw[y_max, x_min] + self.depth_raw[y_max, x_max] + self.depth_raw[y_min, x_max])/4
                depth_gap = abs(precise_cz - check_point_depth_mean)
                if depth_gap < 5:
                    continue

                stack = False
                if not self.only_blocks:
                    # detect all kinds of blocks
                    if len(approx) > 5 and depth_gap > 40:
                        type = "cylinder"
                    elif len(approx) == 5 and box_width > 40:
                        type = "arch"
                    elif len(approx) == 5 and box_width <= 40:
                        type = "up-semi-circle"
                    elif len(approx) == 3:
                        type = "triangle"
                    elif len(approx) == 4 and box_width < 35:
                        type = "small"
                        if depth_gap > 30:
                            stack = True
                    elif len(approx) == 4 and box_width >= 35:
                        type = "big"
                        if depth_gap > 50:
                            stack = True
                else:
                    if box_width < 35:
                        type = "small"
                        if depth_gap > 30:
                            stack = True
                    else:
                        type = "big"
                        if depth_gap > 50:
                            stack = True

                a_block = block(center=[precise_cy, precise_cx], depth=depth_gap, orientation=box_angle, contour=precise_contour, color=color, type=type, stack=stack)
                self.blocks.append(a_block)
                break


    def _refine_contour(rgb_img, depth_raw, n_colors, useHsv = False):
        """!
        @brief      Utilize clustering for refining each subregion.

        @param      rgb_img: subregion of RGB image.
                    depth_raw: subregion of depth image.
                    useHsv: Use hsv as a characteristic vector.
                    n_colors: number of clusters.
        
        @return     success: True or False
                    binary_image: refined region after cluster.
        """
        lab_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2LAB)

        depth_img = cv2.normalize(depth_raw, None, 0, 1, cv2.NORM_MINMAX)
        rgb_pixels = rgb_img.reshape((-1, 3))
        lab_pixels = lab_img.reshape((-1, 3))
        depth_pixels = depth_img.reshape((-1, 1))
        if useHsv:
            hsv_img = cv2.cvtColor(rgb_img, cv2.COLOR_RGB2HSV)
            hsv_pixels = hsv_img.reshape((-1, 3))
            combined_pixels = np.hstack((rgb_pixels, lab_pixels, hsv_pixels, depth_pixels))
        else:
            combined_pixels = np.hstack((rgb_pixels, lab_pixels, depth_pixels))
        
        kmeans = KMeans(n_clusters=n_colors)
        kmeans.fit(combined_pixels)

        label_counts = Counter(kmeans.labels_)

        blank_image = np.ones(rgb_img.shape, dtype=np.uint8) * 255

        if len(label_counts.most_common()) > 2:
            most_common_label = label_counts.most_common(3)[1][0]
            second_common_label = label_counts.most_common(3)[2][0]

            most_common_count = label_counts.most_common(3)[1][1]
            second_common_count = label_counts.most_common(3)[2][1] 

            # Combine
            if most_common_count < 400:
                kmeans.labels_[kmeans.labels_ == second_common_label] = most_common_label
        else:
            return False, None

        blank_image = np.ones(rgb_img.shape, dtype=np.uint8) * 255
        cluster_indices = np.where(kmeans.labels_ == most_common_label)
        blank_image.reshape((-1, 3))[cluster_indices] = kmeans.cluster_centers_[most_common_label, :3].astype(np.uint8)

        gray_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 254, 255, cv2.THRESH_BINARY_INV)

        return True, binary_image


    def _detect_color(self, frame_rgb, contour):
        # color range
        frame_lab = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2LAB)
        frame_hsv = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2HSV)

        color_rgb_mean = np.array([[127, 19, 30], #red
                                    [164, 66, 5], #orange
                                    [218, 180, 30], #yellow
                                    [30, 110, 60], # green
                                    [5, 60, 110], # blue
                                    [50, 50, 73]], # purple
                                    dtype=np.uint8)
        color_lab_mean = cv2.cvtColor(color_rgb_mean[:,None,:], cv2.COLOR_RGB2LAB).squeeze()
        color_hsv_mean = cv2.cvtColor(color_rgb_mean[:,None,:], cv2.COLOR_RGB2HSV).squeeze()
        color_mean = np.concatenate((color_rgb_mean, color_lab_mean, color_hsv_mean), axis=1)
    
        # RGB features
        mask_rgb = np.zeros(frame_rgb.shape[:2], dtype="uint8")
        cv2.drawContours(mask_rgb, [contour], -1, 255, cv2.FILLED)
        mean_rgb = np.array(cv2.mean(frame_rgb, mask=mask_rgb)[:3], dtype=np.float32)


        # LAB features
        mask_lab = np.zeros(frame_lab.shape[:2], dtype="uint8")
        cv2.drawContours(mask_lab, [contour], -1, 255, cv2.FILLED)
        mean_lab = np.array(cv2.mean(frame_lab, mask=mask_lab)[:3], dtype=np.float32)


        # HSV features
        mask_hsv = np.zeros(frame_hsv.shape[:2], dtype="uint8")
        cv2.drawContours(mask_hsv, [contour], -1, 255, cv2.FILLED)
        mean_hsv = np.array(cv2.mean(frame_hsv, mask=mask_hsv)[:3], dtype=np.float32)


        features = np.concatenate((mean_rgb, mean_lab, mean_hsv))
        dist = color_mean - features
        data = features.tolist()


        with open('../data/models/model.pkl', 'rb') as f:
            model = pickle.load(f)
        pred = model.predict(features[None, :])
        data.append(int(pred))
        if int(pred) == 0:
            return "red"
        elif int(pred) == 1:
            return "orange"
        elif int(pred) == 2:
            return "yellow"
        elif int(pred) == 3:
            return "green"
        elif int(pred) == 4:
            return "blue"
        elif int(pred) == 5:
            return "purple"
        else:
            return "unknown"





def drawblock(blocks, output_img:np.array, boundary = None) -> np.array:
    """!
    @brief      Draw blocks for visualization


    @param      block: information of a block
                img: output image for visualization
    """
    for block in blocks:
        color = block.color
        orientation = block.orientation
        center = block.center[1], block.center[0]
        cv2.circle(output_img, (center[0], center[1]), 2, (0,255,0), -1)
        cv2.putText(output_img, str(int(np.rad2deg(orientation))), (center[0], center[1]), font, 0.4, (0,255,0), thickness=1)
        cv2.putText(output_img, str(color), (center[0] + 15, center[1] - 15), font, 0.4, (0,255,0), thickness=1)
        cv2.putText(output_img, block.type, (center[0] + 15, center[1] + 15), font, 0.4, (0,255,0), thickness=1)
        if block.stack:
            cv2.putText(output_img, "stack", (center[0] + 15, center[1]), font, 0.4, (0,255,0), thickness=1)
        cv2.drawContours(output_img, [block.contour], -1, (255,0,0), 2)
    
    
    
    if boundary != None:
        for item in boundary:
            if item[1] == 1:
                cv2.fillPoly(output_img, [item[0]], 255)
            else:
                cv2.fillPoly(output_img, [item[0]], 0)

    cv2.imwrite("../data/result.png", output_img)
    
    return output_img