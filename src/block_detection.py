import numpy as np
import cv2

def blockDetector(img):
    """!
    @brief      Detect blocks from rgb

                TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                locations in self.block_detections
    """

    # load the image
    img=cv2.imread(img)
    # frame = cv2.resize(cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE),(640,480))
    frame = img
    result_image = np.zeros_like(frame)

    # red
    lower_red_1 = np.array([0, 43, 46])
    higher_red_1 = np.array([10, 255, 255])
    lower_red_2 = np.array([156, 43, 46])
    higher_red_2 = np.array([180, 255, 255])

    # orange
    lower_orange = np.array([11, 43, 46])
    higher_orange = np.array([25, 255, 255])

    # yellow
    lower_yellow = np.array([26, 43, 46])
    higher_yellow = np.array([34, 255, 255])

    # green
    lower_green = np.array([35, 43, 46])
    higher_green = np.array([77, 255, 255])

    # blue
    lower_blue = np.array([100, 43, 46])
    higher_blue = np.array([105, 255, 255]) 

    # purple
    lower_purple = np.array([106, 43, 46])
    higher_purple = np.array([150, 255, 255])  

    # pink
    lower_pink = np.array([165, 43, 46])
    higher_pink = np.array([255, 255, 255])   

    img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask_red_1 = cv2.medianBlur(cv2.inRange(img_hsv, lower_red_1, higher_red_1), 7)
    mask_red_2 = cv2.medianBlur(cv2.inRange(img_hsv, lower_red_2, higher_red_2), 7)
    mask_orange = cv2.medianBlur(cv2.inRange(img_hsv, lower_orange, higher_orange), 7)
    mask_yellow = cv2.medianBlur(cv2.inRange(img_hsv, lower_yellow, higher_yellow), 7)
    mask_green = cv2.medianBlur(cv2.inRange(img_hsv, lower_green, higher_green), 7)
    mask_blue = cv2.medianBlur(cv2.inRange(img_hsv, lower_blue, higher_blue), 7)
    mask_purple = cv2.medianBlur(cv2.inRange(img_hsv, lower_purple, higher_purple), 7)
    mask_pink = cv2.medianBlur(cv2.inRange(img_hsv, lower_pink, higher_pink), 7)

    result_image[mask_red_1 > 0] = [255, 255, 255]
    result_image[mask_red_2 > 0] = [255, 255, 255]
    result_image[mask_orange > 0] = [255, 255, 255]
    result_image[mask_yellow > 0] = [255, 255, 255]
    result_image[mask_green > 0] = [255, 255, 255]
    result_image[mask_blue > 0] = [255, 255, 255]
    result_image[mask_purple > 0] = [255, 255, 255]
    result_image[mask_pink > 0] = [255, 255, 255]

    cv2.imshow('Image', mask_purple)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    

def detectBlocksInDepthImage(rgb_img,depth_img):
    """!
    @brief      Detect blocks from depth

                TODO: Implement a blob detector to find blocks in the depth image
    """
    depth_img = cv2.imread(depth_img,cv2.IMREAD_GRAYSCALE)
    edges = cv2.Canny(depth_img.astype(np.uint8), 20, 30)
    kernel_size = 5
    blurred_edges = cv2.GaussianBlur(edges,(kernel_size,kernel_size),0)
    cv2.imshow("Blurred edges",blurred_edges)
    cv2.imshow("original edges",edges)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rgb_img = '../data/Deapth_RGB/RGB_image3.png'
    depth_img = '../data/Deapth_RGB/depth_image3.png'
    detectBlocksInDepthImage(rgb_img,depth_img)