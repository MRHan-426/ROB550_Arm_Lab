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


def new_detectBlocksColorInRGBImage(frame_rgb, contour):
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


def detectBlocksUsingCluster(rgb_img, depth_raw, boundary, only_blocks = True, lower = 850, upper = 1100):
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
    blocks = []
    # 1. Filter by height to eliminate the robot arm.
    height_filter = cv2.inRange(depth_raw, lower, upper)
    mask = cv2.merge([height_filter, height_filter, height_filter])
    rgb_img_remove_robot_arm = cv2.bitwise_and(rgb_img, mask)

    # 2. Filter the grid through hsv frame.
    hsv_img = cv2.cvtColor(rgb_img_remove_robot_arm, cv2.COLOR_RGB2HSV)
    hsv_mask = cv2.inRange(hsv_img, np.array([0,43,46]), np.array([180, 255, 255]))
    # hsv_mask = cv2.medianBlur(hsv_mask, 3)
    boundary_mask = np.zeros_like(hsv_mask, dtype=np.uint8)
    if boundary != []:
        if len(boundary) == 2:
            cv2.fillPoly(boundary_mask, [boundary[0]], 255)
            cv2.fillPoly(boundary_mask, [boundary[1]], 0)
        else:
            cv2.fillPoly(boundary_mask, [boundary[0]], 255)
            cv2.fillPoly(boundary_mask, [boundary[1]], 0)
            cv2.fillPoly(boundary_mask, [boundary[2]], 0)

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

        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cz_0 = depth_raw[cy, cx]
        cz_1 = depth_raw[cy + 8, cx + 8]
        cz_2 = depth_raw[cy + 8, cx - 8]
        cz_3 = depth_raw[cy - 8, cx + 8]
        cz_4 = depth_raw[cy - 8, cx - 8]
        cz_5 = depth_raw[cy + 4, cx + 4]
        cz_6 = depth_raw[cy + 4, cx - 4]
        cz_7 = depth_raw[cy - 4, cx + 4]
        cz_8 = depth_raw[cy - 4, cx - 4]
        cz = min(cz_0, cz_1, cz_2, cz_3, cz_4, cz_5, cz_6, cz_7, cz_8) # choose the highest

        y_min = int(cy - sub_region_side / 2)
        y_max = int(cy + sub_region_side / 2)
        x_min = int(cx - sub_region_side / 2)
        x_max = int(cx + sub_region_side / 2)
        if x_min < 0 or y_min < 0:
            x_min = 0
            y_min = 0
        if x_max > 1280:
            x_max = 1280
        if y_max > 720:
            y_max = 720

        canvas = np.zeros_like(masked_hsv)
        cv2.drawContours(canvas, [raw_contour], 0, 255, -1)
        result = cv2.bitwise_and(rgb_img, rgb_img, mask=canvas)
        sub_region = result[y_min:y_max, x_min:x_max]
        sub_region_depth = depth_raw[y_min:y_max, x_min:x_max]

        check_point_depth_mean = (depth_raw[y_min,x_min] + depth_raw[y_max, x_min] + depth_raw[y_max, x_max] + depth_raw[y_min, x_max])/4
        depth_gap = abs(cz - check_point_depth_mean)

        # 4. Perform depth-based detection to determine if it forms a stack.
        if depth_gap > 42:
            # This is a stack
            stack = True
            # filter the lower level
            height_filter = cv2.inRange(sub_region_depth, int(cz) - 20, int(cz) + 10)
            mask = cv2.merge([height_filter, height_filter, height_filter])
            sub_region = cv2.bitwise_and(sub_region, mask)
            n_colors = 3

        elif depth_gap < 5:
            continue
        else:
            stack = False
            n_colors = 4

        # 5. Utilize clustering for refining each subregion.
        success, refined_region = clusterThroughRgbLabDepth(sub_region, sub_region_depth, n_colors)
        
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
            if not only_blocks:
                if len(approx) != 4:
                    type = "unknown"
                else:
                    type = "blocks"
            else:
                type = "blocks"

            (box_x, box_y), (box_width, box_height), box_angle = cv2.minAreaRect(approx)
            if box_width < box_height:
                box_width, box_height = box_height, box_width
            if box_height < 10:
                continue
            aspect_ratio = box_width / box_height
        
            precise_cx = int(M['m10']/M['m00'])
            precise_cy = int(M['m01']/M['m00'])
            precise_cz = sub_region_depth[precise_cy, precise_cx]

            if not only_blocks:
                if type != "unknown":
                    if abs(aspect_ratio - 1) > 0.4 and not stack and M["m00"] > 850:
                        type = "unknown"
                    elif M["m00"] < 850:
                        type = "small"
                    else:
                        type = "big"
            else:
                if M["m00"] < 850:
                    type = "small"
                else:
                    type = "big"

            precise_cx = precise_cx + cx - int(sub_region_side/2)
            precise_cy = precise_cy + cy - int(sub_region_side/2)
            precise_contour = precise_contour + np.array([cx - int(sub_region_side/2), cy - int(sub_region_side/2)])

            color = new_detectBlocksColorInRGBImage(rgb_img, precise_contour)
            
            y_min = int(precise_cy - sub_region_side / 2)
            y_max = int(precise_cy + sub_region_side / 2)
            x_min = int(precise_cx - sub_region_side / 2)
            x_max = int(precise_cx + sub_region_side / 2)
            check_point_depth_mean = (depth_raw[y_min,x_min] + depth_raw[y_max, x_min] + depth_raw[y_max, x_max] + depth_raw[y_min, x_max])/4
            depth_gap = abs(cz - check_point_depth_mean)
            if depth_gap > 30 and type == "small":
                # This is a stack
                stack = True
                print(color, f"precise depth gap = {depth_gap}")
            elif depth_gap > 50 and type == "big":
                stack = True
                print(color, f"precise depth gap = {depth_gap}")
            
            elif depth_gap < 5:
                continue
            else:
                stack = False

            a_block = block(center=[precise_cy, precise_cx], depth=depth_gap, orientation=box_angle, contour=precise_contour, color=color, type=type, stack=stack)
            blocks.append(a_block)
            break
    return blocks


def clusterThroughRgbLabDepth(rgb_img, depth_raw, n_colors, useHsv = False):
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
        if len(boundary) == 2:
            cv2.fillPoly(output_img, [boundary[0]], 255)
            cv2.fillPoly(output_img, [boundary[1]], 0)
        elif len(boundary) == 3:
            cv2.fillPoly(output_img, [boundary[0]], 255)
            cv2.fillPoly(output_img, [boundary[1]], 0)
            cv2.fillPoly(output_img, [boundary[2]], 0)

    cv2.imwrite("../data/result.png", output_img)
    
    return output_img