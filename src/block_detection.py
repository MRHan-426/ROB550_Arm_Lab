import numpy as np
import cv2

font = cv2.FONT_HERSHEY_SIMPLEX

def blockDetector(img):
    """!
    @brief      Detect blocks from rgb

                TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                locations in block_detections
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
    depth_img = cv2.imread(depth_img, cv2.IMREAD_UNCHANGED)
    rgb_img = cv2.imread(rgb_img)
    INTRINISC_MATRIX = np.array([
                        [918.2490195188435, 0.0, 636.4533753942957],
                        [0.0, 912.0611927215057, 365.23840749139805],
                        [0.0, 0.0, 1.0]])
    extrinsic_matrix = np.array([
                        [ 9.99581091e-01, -2.82705457e-02, -6.19823419e-03, 3.45427247e+01],
                        [-2.62323435e-02, -9.75452033e-01, 2.18643994e-01, 1.32930439e+02],
                        [-1.22272652e-02, -2.18389808e-01, -9.75785010e-01, 1.04360917e+03],
                        [ 0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
                        ])
    y = np.arange(depth_img.shape[0])
    x = np.arange(depth_img.shape[1])
    mesh_x, mesh_y = np.meshgrid(x, y)
    Z = depth_img.astype(np.float32)
    X = (mesh_x - INTRINISC_MATRIX[0, 2]) * Z / INTRINISC_MATRIX[0, 0]
    Y = (mesh_y - INTRINISC_MATRIX[1, 2]) * Z / INTRINISC_MATRIX[1, 1]

    homogeneous_coordinates = np.stack((X, Y, Z, np.ones_like(Z)), axis=-1)
    P_c = homogeneous_coordinates.reshape(-1, 4).T
    P_w = np.linalg.inv(extrinsic_matrix) @ P_c
    points_3d = P_w.T[:, 2].reshape(depth_img.shape[0], depth_img.shape[1], 1)

    cv2.imwrite("affline_depth.png", points_3d)
    depth_data2 = cv2.imread("affline_depth.png")
    # y = np.arange(rgb_img.shape[0]) #720
    # x = np.arange(rgb_img.shape[1])
    # mesh_x, mesh_y = np.meshgrid(x, y)
    # Z = depth_img.astype(np.float32)
    # X = (mesh_x - INTRINISC_MATRIX[0, 2]) * Z / INTRINISC_MATRIX[0, 0]
    # Y = (mesh_y - INTRINISC_MATRIX[1, 2]) * Z / INTRINISC_MATRIX[1, 1]

    # homogeneous_coordinates = np.stack((X, Y, Z, np.ones_like(Z)), axis=-1)
    # P_c = homogeneous_coordinates.reshape(-1, 4).T
    # P_w = np.linalg.inv(extrinsic_matrix) @ P_c
    # P_w = P_w.reshape((rgb_img.shape[0], rgb_img.shape[1], 4))

    # cv2.imwrite("affline_rgb.png", points_3d)

    # cv2.imshow("jb",points_3d)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    
    depth_data = points_3d
    cnt_image = rgb_img
    lower = 10
    upper = 50

    
    print(points_3d.shape)
    print(depth_data2.shape)

    mask = np.zeros_like(depth_data, dtype=np.uint8)
    cv2.rectangle(mask, (275,120),(1100,720), 255, cv2.FILLED)
    cv2.rectangle(mask, (575,414),(723,720), 0, cv2.FILLED)
    cv2.rectangle(depth_data2, (275,120),(1100,720), (255, 0, 0), 2)
    cv2.rectangle(depth_data2, (575,414),(723,720), (255, 0, 0), 2)
    thresh = cv2.bitwise_and(cv2.inRange(depth_data, lower, upper), mask)
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    # cv2.drawContours(depth_data2, contours, -1, (0,255,255), thickness=1)

    for contour in contours:
        cv2.drawContours(depth_data2, contour, -1, (0,255,255), thickness = 1)
        theta = cv2.minAreaRect(contour)[2]
        M = cv2.moments(contour)
        print(M)
        cx = 1
        cy = 1
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        cv2.putText(depth_data2, str(int(theta)), (cx, cy), font, 0.5, (0,255,255), thickness=2)

    cv2.imshow("Image window", depth_data2)
    cv2.waitKey(0)


 


    # edges = cv2.Canny(depth_img.astype(np.uint8), 20, 30)
    # kernel_size = 5
    # blurred_edges = cv2.GaussianBlur(edges,(kernel_size,kernel_size),0)
    # cv2.imshow("Blurred edges",blurred_edges)
    # cv2.imshow("original edges",edges)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()



if __name__ == '__main__':
    rgb_img = '../data/Deapth_RGB/RGB_image3.png'
    depth_img = '../data/Deapth_RGB/depth_image3.png'
    # rgb_img = 'C:\Users\Lynx1\Desktop\MS\A-Course\ROB 550\ROB550\data\Deapth_RGB\depth_image3.png'
    # depth_img = 'C:\Users\Lynx1\Desktop\MS\A-Course\ROB 550\ROB550\data\Deapth_RGB\depth_image3.png'
    detectBlocksInDepthImage(rgb_img,depth_img)
   