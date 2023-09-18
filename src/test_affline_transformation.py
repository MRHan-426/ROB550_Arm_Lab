import cv2
import numpy as np

INTRINISC_MATRIX = np.array([[918.2490195188435, 0.0, 636.4533753942957],
                            [0.0, 912.0611927215057, 365.23840749139805],
                            [0.0, 0.0, 1.0]])
extrinsic_matrix = np.array([[ 9.99925603e-01,-9.82961653e-03 ,7.22269706e-03 , 2.52173618e+01],
                            [-1.10674474e-02 ,-9.80057090e-01 , 1.98407693e-01 , 1.39052668e+02],
                            [ 5.12838392e-03 ,-1.98472869e-01, -9.80092965e-01 , 1.04638438e+03],
                            [ 0.00000000e+00 , 0.00000000e+00 , 0.00000000e+00 , 1.00000000e+00]])

grid_x_points = np.arange(-500, 501, 50)
grid_y_points = np.arange(-175, 525, 50)

def matrix_calculation(img):
    X, Y = np.meshgrid(grid_x_points, grid_y_points)
    grid_points = np.vstack((X.ravel(), Y.ravel(), np.ones_like(X).ravel(), np.ones_like(X).ravel())).astype(np.float32)
    camera_points = np.dot(extrinsic_matrix[0:3, :], grid_points)
    pixel_points = np.dot(INTRINISC_MATRIX, camera_points)
    pixel_x = pixel_points[0, :] / pixel_points[2, :]
    pixel_y = pixel_points[1, :] / pixel_points[2, :]
    for px, py in zip(pixel_x, pixel_y):
        point_pos = (int(px), int(py))
        cv2.circle(modified_image, point_pos, 3, (0, 255, 0), thickness=-1)

    cv2.imshow('Image', modified_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

def double_for_operation(img):
    for x in grid_x_points:
        for y in grid_y_points:
            world_point = np.array([x,y,1,1]).reshape(4,1).astype(np.float32)
            camera_point = np.dot(extrinsic_matrix[0:3,:], world_point) # 3x1
            pixel_point = np.dot(INTRINISC_MATRIX, camera_point)
            pixel_x = pixel_point[0] / pixel_point[2]
            pixel_y = pixel_point[1] / pixel_point[2]
            # TODO: Precise
            point_pos = [int(pixel_x), int(pixel_y)]
            cv2.circle(modified_image, point_pos, 3, (0, 255, 0), thickness=-1)

    cv2.imshow('Image', modified_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    modified_image = cv2.imread("data/2.jpg")
    # matrix_calculation(modified_image)
    double_for_operation(modified_image)

    
