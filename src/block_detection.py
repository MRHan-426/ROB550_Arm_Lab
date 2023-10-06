import numpy as np
import cv2
from dataclasses import dataclass
from scipy import stats
import pickle
import warnings
warnings.filterwarnings("ignore", message="Trying to unpickle estimator SVC from version", category=UserWarning)

font = cv2.FONT_HERSHEY_SIMPLEX


INTRINISC_MATRIX = np.array([
                    [918.2490195188435, 0.0, 636.4533753942957],
                    [0.0, 912.0611927215057, 365.23840749139805],
                    [0.0, 0.0, 1.0]])


EXTRINSIC_MATRIX = np.array([
                    [ 9.99581091e-01, -2.82705457e-02, -6.19823419e-03, 3.45427247e+01],
                    [-2.62323435e-02, -9.75452033e-01, 2.18643994e-01, 1.32930439e+02],
                    [-1.22272652e-02, -2.18389808e-01, -9.75785010e-01, 1.04360917e+03],
                    [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
                    ])




class block:
    def __init__(self, center, depth, contour, orientation, color = None, side = None):
        self.center = center # height, width
        self.depth = depth
        self.color = color
        self.side = side
        self.contour = contour
        self.orientation = np.deg2rad(orientation)

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


        # # Calculate vectors from point to each vertex of the square
        # vectors = []
        # for vertex in self.vertices:
        #     vx, vy = vertex
        #     vectors.append((vx - x, vy - y))

        # # Calculate the cross product of consecutive vectors
        # cross_products = []
        # for i in range(len(vectors)):
        #     x1, y1 = vectors[i]
        #     x2, y2 = vectors[(i + 1) % len(vectors)]
        #     cross_products.append(x1 * y2 - x2 * y1)

        # # If all cross products have the same sign, the point is inside the square
        # return all(cp >= 0 for cp in cross_products) or all(cp <= 0 for cp in cross_products)
   

    def colordetection(self,img):
        detected_color = detectBlocksColorInRGBImage(img,tuple(self.center))
        return detected_color
        # find 5 points to be detected
        # x,y = self.center
        # points = []
        # points.append(self.center)
        # for vertex in self.vertices:
        #     vx, vy = vertex
        #     vector = (vx - x, vy - y)
        #     points.append((int(x + vector[0]/2), int(y + vector[1]/2)))
       
        # color_counts = {'red':0,'blue':0,'yellow':0,'green':0,'orange':0,'purple':0,'pink':0,'unknown':0}
        # for point in points:
        #     detected_color = detectBlocksColorInRGBImage(img,tuple(point))
        #     color_counts[detected_color] += 1
        # max_color = max(color_counts, key=color_counts.get)
        # return max_color



def detectBlocksColorInRGBImage(img, position: tuple) -> str:
    """!
    @brief      Detect blocks from rgb
   
    @param      position x, y
               
                return image


    """
    # cv2.imwrite("temp.jpg", img)
    # img = cv2.imread("temp.jpg")


    img_hsv = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)


    masks = {
            "red": (
                cv2.inRange(img_hsv, np.array([0, 43, 46]), np.array([20, 255, 255])) +
                cv2.inRange(img_hsv, np.array([156, 43, 46]), np.array([190, 255, 255]))
                    ),
            "orange":cv2.inRange(img_hsv, np.array([11,43,46]), np.array([25, 255, 255])),
            "yellow":cv2.inRange(img_hsv, np.array([26,43,46]), np.array([34, 255, 255])),
            # "orange": cv2.medianBlur(cv2.inRange(img_hsv, np.array([11, 43, 46]), np.array([25, 255, 255])), 7),
            # "yellow": cv2.medianBlur(cv2.inRange(img_hsv, np.array([26, 43, 46]), np.array([34, 255, 255])), 7),
            "green": cv2.inRange(img_hsv, np.array([60, 43, 46]), np.array([95, 255, 255])),
            "blue": cv2.inRange(img_hsv, np.array([100, 43, 46]), np.array([105, 255, 255])),
            "purple":cv2.inRange(img_hsv, np.array([106,43,46]), np.array([150, 255, 255])),
            # "purple": cv2.medianBlur(cv2.inRange(img_hsv, np.array([106, 43, 46]), np.array([150, 255, 255])), 7),
            # "pink": cv2.medianBlur(cv2.inRange(img_hsv, np.array([165, 43, 46]), np.array([255, 255, 255])), 7)
        }


    detected_color = "unknown"
    for color, mask in masks.items():
        if mask[position[0], position[1]] > 0:
            detected_color = color
            break


    # debug
    if __name__ == '__main__':
        cv2.circle(img, position, 5, (0, 0, 255), -1) # Red circle of radius 5
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.5
        font_thickness = 1
        text_position = (position[0] + 10, position[1])
        cv2.putText(img, detected_color, text_position, font, font_scale, (0, 0, 255), font_thickness)
        cv2.imshow('Image', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


    return detected_color


def detectBlocksInDepthImage(depth_img, intrinsic_matrix = INTRINISC_MATRIX, extrinsic_matrix = EXTRINSIC_MATRIX):
    """!
    @brief      Detect blocks from depth


                Implement a blob detector to find blocks in the depth image


    @param      depth_img: 720x1280x1 raw depth image
                intrinsic and extrinsic of camera
               
                return: a list of blocks
    """
    if isinstance(depth_img, str):
        depth_img = cv2.imread(depth_img, cv2.IMREAD_UNCHANGED)
       
    depth_data = depth_img


    #  using depth image to detect contour
    lower = 10
    upper = 40
    mask = np.zeros_like(depth_data, dtype=np.uint8)
    cv2.rectangle(mask, (40,40),(1280,720), 255, cv2.FILLED)
    # cv2.rectangle(mask, (600,360),(730,710), 0, cv2.FILLED)
    thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


    blocks = []


    for contour in contours:
        epsilon = 0.08 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)
        # print("approx.shape is: ", approx.shape)
        if len(approx) == 4:
            point1 = [approx[0,0,0],approx[0,0,1]]
            point2 = [approx[2,0,0],approx[2,0,1]]
            diagonal_length = np.sqrt(np.square(point1[0]-point2[0])+np.square(point1[1]-point2[1]))
            if diagonal_length > 20 and diagonal_length < 60:            
                orientation = cv2.minAreaRect(contour)[2]
                M = cv2.moments(contour)
                cy = 1
                cx = 1
                if M['m00'] != 0:
                    cy = int(M['m01']/M['m00'])
                    cx = int(M['m10']/M['m00'])
                # drawblock([cx,cy], diagonal_length, orientation, output_img)


                # if diagonal_length > 40:
                    # side = 40
                    # cv2.putText(output_img, "big block", (cx + 5, cy + 5), font, 0.4, (0,255,0), thickness=1)


                # else:
                    # side = 20
                    # cv2.putText(output_img, "small block", (cx + 5, cy + 5), font, 0.4, (0,255,0), thickness=1)


                # if diagonal_length > 40:
                #     side = 40
                # else:
                #     side = 20


                a_block = block([cy,cx] , depth_data[cy, cx], diagonal_length/np.sqrt(2), orientation)
                blocks.append(a_block)


                # cv2.circle(depth_data2, (cx,cy), 2, (0,255,0), -1)
                # cv2.putText(depth_data2, str(int(orientation)), (cx, cy), font, 0.4, (0,255,0), thickness=1)


    # block_counter = 1
    # for a_block in blocks:
    #     print("Block NO.", block_counter, " | side: ", a_block.side,  " | position: ", a_block.center[0],", ", a_block.center[1])
    #     block_counter += 1


    # debug
    if __name__ == '__main__':
        cv2.imshow("Image window", depth_data)
        cv2.waitKey(0)
        # print("block size is: ", len(blocks))
   
    return blocks
 
   
def detectBlocksInDepthImageCanny():
    pass
    # # Edge Detection Using Canny
    # cv2.imwrite("affline_depth.png", points_3d)
    # points_3d = cv2.imread("affline_depth.png", cv2.IMREAD_GRAYSCALE)
    # kernel_size = 3
    # blurred_edges = cv2.GaussianBlur(points_3d,(kernel_size,kernel_size), 10)
    # cv2.imshow("Blurred edges",blurred_edges)
    # blurred_edges = cv2.Canny(blurred_edges,10, 25,apertureSize=3,L2gradient=True)
   
    # lower = -1
    # upper = 2
    # mask = np.zeros_like(blurred_edges, dtype=np.uint8)
    # cv2.rectangle(mask, (250,80),(1100,720), 255, cv2.FILLED)
    # cv2.rectangle(mask, (600,360),(730,710), 0, cv2.FILLED)
    # cv2.rectangle(blurred_edges, (250,80),(1100,720), (255, 0, 0), 2)
    # cv2.rectangle(blurred_edges, (600,360),(730,710), (255, 0, 0), 2)
    # thresh = cv2.bitwise_and(cv2.inRange(blurred_edges, lower, upper), mask)
    # contours, _ = cv2.findContours(blurred_edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(rgb_img, contours, -1, (0,255,0), thickness = -1)


    # for contour in contours:
    #     epsilon = 0.1 * cv2.arcLength(contour, True)
    #     approx = cv2.approxPolyDP(contour, epsilon, True)
    #     if len(approx) == 2:
    #         cv2.drawContours(rgb_img, contour, -1, (0,255,255), thickness = 1)
    #         orientation = cv2.minAreaRect(contour)[2]
    #         M = cv2.moments(contour)
    #         cx = 1
    #         cy = 1
    #         if M['m00'] != 0:
    #             cx = int(M['m10']/M['m00'])
    #             cy = int(M['m01']/M['m00'])
    #         cv2.circle(rgb_img,(cx,cy),3,(0,255,0),-1)
    #         cv2.putText(rgb_img, str(int(orientation)), (cx, cy), font, 0.5, (0,255,0), thickness=1)


    # cv2.imshow("original edges",rgb_img)
    # print("blurred img shape is: ", blurred_edges)


    # cv2.waitKey(0)
    # cv2.destroyAllWindows()




# draw the block on the image
def drawblock(blocks, output_img:np.array, boundary = None, new:bool = False) -> np.array:
    """!
    @brief      Draw blocks for visualization


    @param      block: information of a block
                img: output image for visualization
    """
    i = 0
    for block in blocks:
        if not new:
            color = block.colordetection(output_img)
            if color == "unknown":
                continue
            half_side = block.side / 2
            orientation = block.orientation
            center = block.center[1], block.center[0]
            cos_angle = np.cos(orientation)
            sin_angle = np.sin(orientation)
            cv2.putText(output_img, color, (center[0] + 15, center[1] - 15), font, 0.4, (0,255,0), thickness=1)

            corner1 = (int(center[0] - half_side * cos_angle - half_side * sin_angle),
                int(center[1] + half_side * cos_angle - half_side * sin_angle))
            corner2 = (int(center[0] + half_side * cos_angle - half_side * sin_angle),
                int(center[1] + half_side * cos_angle + half_side * sin_angle))
            corner3 = (int(center[0] + half_side * cos_angle + half_side * sin_angle),
                int(center[1] - half_side * cos_angle + half_side * sin_angle))
            corner4 = (int(center[0] - half_side * cos_angle + half_side * sin_angle),
                int(center[1] - half_side * cos_angle - half_side * sin_angle))
            square_coordinates = np.array([corner1, corner2, corner3, corner4], dtype=np.int32)
            # RGB
            cv2.polylines(output_img, [square_coordinates], isClosed=True, color=(255, 255, 0), thickness=2)
            cv2.circle(output_img, (center[0], center[1]), 2, (0,255,0), -1)

            cv2.putText(output_img, str(int(np.rad2deg(orientation))), (center[0], center[1]), font, 0.4, (0,255,0), thickness=1)
           
            if block.side > 30:
                cv2.putText(output_img, "big block", (center[0] + 15, center[1] + 15), font, 0.4, (0,255,0), thickness=1)
            else:
                cv2.putText(output_img, "small block", (center[0] + 15, center[1] + 15), font, 0.4, (0,255,0), thickness=1)
        else:
            color = block.color
            orientation = block.orientation
            center = block.center[1], block.center[0]
            cos_angle = np.cos(orientation)
            sin_angle = np.sin(orientation)
            cv2.putText(output_img, str(color), (center[0] + 15, center[1] - 15), font, 0.4, (0,255,0), thickness=1)
            cv2.circle(output_img, (center[0], center[1]), 2, (0,255,0), -1)
            cv2.putText(output_img, str(int(np.rad2deg(orientation))), (center[0], center[1]), font, 0.4, (0,255,0), thickness=1)
            cv2.drawContours(output_img, block.contour, -1, (255,0,0), 2)
    
    if boundary != None:
        cv2.fillPoly(output_img, [boundary[0]], 255)
        cv2.fillPoly(output_img, [boundary[1]], 0)
    return output_img


def new_detectBlocksInDepthImage(depth_raw, output_img, boundary, blind_rect=None):
    lower = 10
    upper = 40

    lab_img = cv2.cvtColor(output_img, cv2.COLOR_RGB2LAB)
    hsv_img = cv2.cvtColor(output_img, cv2.COLOR_RGB2HSV)

    depth_raw = depth_raw[:,:,0].astype('uint8')

    depth_raw = cv2.medianBlur(depth_raw, 3)
    mask = np.zeros_like(depth_raw, dtype=np.uint8)
    pts1 = boundary[0]
    pts2 = boundary[1]
    cv2.fillPoly(mask, [pts1], 255)
    cv2.fillPoly(mask, [pts2], 0)

    # cv2.rectangle(mask, (0, 90), (1083, 700), 255, cv2.FILLED)
    # cv2.rectangle(mask, (570, 400),(735, 700), 0, cv2.FILLED)
    if blind_rect is not None:
        cv2.rectangle(mask, blind_rect[0], blind_rect[1], 0, cv2.FILLED)

    depth_seg = cv2.inRange(depth_raw, lower, upper)
    img_depth_thr = cv2.bitwise_and(depth_seg, mask)
    contours, _ = cv2.findContours(img_depth_thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]
   
    blocks = []

    for contour in contours:
        M = cv2.moments(contour)
        if M['m00'] < 500 or abs(M["m00"]) > 7000:
            continue
        mask_single = np.zeros_like(depth_raw, dtype=np.uint8)
        cv2.drawContours(mask_single, [contour], -1, 255, cv2.FILLED)

        depth_single = cv2.bitwise_and(depth_raw, depth_raw, mask=mask_single)
        depth_array = depth_single[depth_single>=lower]

        mode_real, _ = stats.mode(depth_array)
        depth_diff =  mode_real - depth_array
        depth_array_inliers = depth_array[depth_diff<8]
       
        mode = np.min(depth_array_inliers)
        depth_new = cv2.inRange(depth_single, lower, int(mode)+5)
        contours_new, _ = cv2.findContours(depth_new, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2:]

        if not contours_new:
            continue
        contours_new_valid = max(contours_new, key=cv2.contourArea) # find the largest contour
        M = cv2.moments(contours_new_valid)
        if abs(M["m00"]) < 200:
            continue
        elif M["m00"] < 850:
            side = 20
        else:
            side = 40
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        cz = depth_raw[cy, cx]
        block_ori = cv2.minAreaRect(contours_new_valid)[2]
        color = new_detectBlocksColorInRGBImage(frame_rgb=output_img, frame_lab=lab_img, frame_hsv=hsv_img, contour=contours_new_valid)
        a_block = block(center=[cy,cx] , depth=cz, contour=contours_new_valid, side=side, color= color,orientation=block_ori)
        blocks.append(a_block)
    print("======================================================")
    print("Block Detection done, Total: ", len(blocks))
    print("======================================================")
    return blocks


def new_detectBlocksColorInRGBImage(frame_rgb, frame_lab, frame_hsv, contour):
    # color range
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


if __name__ == '__main__':
    rgb_img = cv2.imread('data/Depth_RGB/rgb_image4.png', cv2.IMREAD_UNCHANGED)
    depth_img = cv2.imread('data/Depth_RGB/depth_image4.png', cv2.IMREAD_UNCHANGED)
    blocks = new_detectBlocksInDepthImage(depth_raw=depth_img)
    output = drawblock(blocks=blocks, output_img=rgb_img, new=True)
    cv2.imshow("a", output)
    cv2.waitKey(0)
    cv2.destroyAllWindows()