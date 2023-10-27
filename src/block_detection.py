import cv2
import numpy as np
from sklearn.cluster import KMeans
from collections import Counter


class block:
    def __init__(self, center, depth, orientation, contour, color, type, stack = False):
        self.center = center # height, width
        self.depth = depth
        self.orientation = np.deg2rad(orientation)
        self.contour = contour
        self.color = color
        self.type = type
        self.stack = stack
        self.world_xyz = np.zeros(3)

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
    def __init__(self, camera):
        self.blocks = []
        self.camera = camera
        self.lower = 850
        self.upper = 1100

        # whole workspace
        self.boundary = [(np.array([[ 200, 75],[ 1100,  75],[1100,  700],[200,  700]]), 1),
                        (np.array([[ 565, 437],[ 735,  437],[735,  700],[565,  700]]), 0)]
        self.only_blocks = True
        self.rgb_img = self.camera.VideoFrame.copy()
        self.depth_raw = self.camera.DepthFrameRaw.copy()
        self.ref_rgb = np.array([[181, 16, 33], # red
                                [213, 99, 23],  # orange
                                [219, 249, 25], # yellow
                                [89, 186, 65],  # green
                                [32, 53, 186],  # blue
                                [85, 14, 107]], # purple
                                dtype=np.uint8)
        self.ref_lab = cv2.cvtColor(self.ref_rgb[:,None,:], cv2.COLOR_RGB2LAB).squeeze()
        self.ref_hsv = cv2.cvtColor(self.ref_rgb[:,None,:], cv2.COLOR_RGB2HSV).squeeze()
        self.ref_vector = np.concatenate((self.ref_rgb, self.ref_lab), axis=1)
        self.ref_vector_hsv = np.concatenate((self.ref_rgb, self.ref_lab, self.ref_hsv), axis=1)
        self.color_names = ["red", "orange", "yellow", "green", "blue", "purple"]
        self.font = cv2.FONT_HERSHEY_SIMPLEX


    def run(self, only_blocks = True, boundary = None, lower = 850, upper = 1100):
        """
        @brief Run the block detection process.
        @param only_blocks: If True, only detect blocks.
        @param boundary: A list containing boundary coordinates.
                            If provided, block detection will be limited within this boundary.
        @param lower: Lower threshold for height filter.
                        Defaults to 850.
        @param upper: Upper threshold for height filter.
                        Defaults to 1100.
        @return None
        """
        self._reset()
        self.only_blocks = only_blocks
        self.lower = lower
        self.upper = upper
        if boundary != None:
            self.boundary = boundary

        self._detect_blocks()
        self._calculate_block_world_coordinate()
        print(f"Block Detection Done! Total {len(self.blocks)} Detected.")


    def drawblock(self, show_boundary = False):
        """!
        @brief      Draw blocks for visualization

        @param      show_boundary: when True, draw self.boundary on the image.

        @return     output: image
        """
        output = self.rgb_img.copy()
        for block in self.blocks:
            cv2.circle(output, (block.center[1], block.center[0]), 2, (0,255,0), -1)
            cv2.putText(output, str(int(np.rad2deg(block.orientation))), (block.center[1], block.center[0]), self.font, 0.4, (0,255,0), thickness=1)
            cv2.putText(output, str(block.color), (block.center[1] + 15, block.center[0] - 15), self.font, 0.4, (0,255,0), thickness=1)
            cv2.putText(output, block.type, (block.center[1] + 15, block.center[0] + 15), self.font, 0.4, (0,255,0), thickness=1)
            if block.stack:
                cv2.putText(output, "stack", (block.center[1] + 15, block.center[0]), self.font, 0.4, (0,255,0), thickness=1)
            cv2.drawContours(output, [block.contour], -1, (255,0,0), 2)
        
        if show_boundary:
            for item in self.boundary:
                if item[1] == 1:
                    cv2.fillPoly(output, [item[0]], 255)
                else:
                    cv2.fillPoly(output, [item[0]], 0)

        return output


    def _reset(self):
        self.blocks = []
        self.rgb_img = self.camera.VideoFrame.copy()
        self.depth_raw = self.camera.DepthFrameRaw.copy()


    def _detect_blocks(self):
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

                    rgb_vector = self.rgb_img[cy, cx].reshape(3, 1)
                    lab_vector = cv2.cvtColor(rgb_vector, cv2.COLOR_RGB2LAB).reshape(3, 1)
                    color_vector = np.vstack((rgb_vector, lab_vector))
                    color = self._detect_color(color_vector, useHsv=False)

                    a_block = block(center=[cy, cx], 
                                    depth=depth_gap, 
                                    orientation=orientation, 
                                    contour=raw_contour, 
                                    color=color, 
                                    type=type, 
                                    stack=stack)
                    self.blocks.append(a_block)
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


    def _refine_contour(self, sub_region, sub_region_depth, n_colors, useHsv = False):
        """!
        @brief      Utilize clustering for refining each subregion.

        @param      sub_region: subregion of RGB image.
                    sub_region_depth: subregion of depth image.
                    useHsv: Use hsv as a characteristic vector.
                    n_colors: number of clusters.
        
        @return     success: True or False
                    binary_image: refined region after cluster.
                    color: detect color
        """
        lab_img = cv2.cvtColor(sub_region, cv2.COLOR_RGB2LAB)
        depth_img = cv2.normalize(sub_region_depth, None, 0, 1, cv2.NORM_MINMAX)
        rgb_pixels = sub_region.reshape((-1, 3))
        lab_pixels = lab_img.reshape((-1, 3))
        depth_pixels = depth_img.reshape((-1, 1))
        if useHsv:
            hsv_img = cv2.cvtColor(sub_region, cv2.COLOR_RGB2HSV)
            hsv_pixels = hsv_img.reshape((-1, 3))
            combined_pixels = np.hstack((rgb_pixels, lab_pixels, hsv_pixels, depth_pixels))
        else:
            combined_pixels = np.hstack((rgb_pixels, lab_pixels, depth_pixels))
        
        kmeans = KMeans(n_clusters=n_colors)
        kmeans.fit(combined_pixels)

        label_counts = Counter(kmeans.labels_)

        blank_image = np.ones(sub_region.shape, dtype=np.uint8) * 255

        if len(label_counts.most_common()) > 2:
            most_common_label = label_counts.most_common(3)[1][0]
            most_common_count = label_counts.most_common(3)[1][1]
            if useHsv:
                most_common_center = kmeans.cluster_centers_[most_common_label, :9]
            else:
                most_common_center = kmeans.cluster_centers_[most_common_label, :6]

            color = self._detect_color(most_common_center, useHsv = False)
        else:
            return False, None, None

        blank_image = np.ones(sub_region.shape, dtype=np.uint8) * 255
        cluster_indices = np.where(kmeans.labels_ == most_common_label)
        blank_image.reshape((-1, 3))[cluster_indices] = kmeans.cluster_centers_[most_common_label, :3].astype(np.uint8)

        gray_image = cv2.cvtColor(blank_image, cv2.COLOR_BGR2GRAY)
        _, binary_image = cv2.threshold(gray_image, 254, 255, cv2.THRESH_BINARY_INV)

        return True, binary_image, color


    def _detect_color(self, input_vector, useHsv = False):
        if useHsv:
            distances = np.linalg.norm(self.ref_vector_hsv - input_vector, axis=1)
        else:
            distances = np.linalg.norm(self.ref_vector - input_vector, axis=1)
        closest_color_idx = np.argmin(distances)
        return self.color_names[closest_color_idx]

        
    def _calculate_block_world_coordinate(self):
        for block in self.blocks:
            x_y = self.camera.transformFromImageToWorldFrame((block.center[1], block.center[0]))[:2]
            block.world_xyz = np.array((x_y[0], x_y[1], block.depth))