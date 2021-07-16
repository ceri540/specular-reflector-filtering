import numpy as np
from spherical import SphericalIntrinsic
from pinhole import PinholeIntrinsic
import cv2
import os

# degree to radian
def D2R(degree):
    return degree * np.math.pi / 180.0  # rotation minus pi equal to degree minus 180.0

# calculate the focal in correspond camera location size
def focal_from_pinhole_intrinsics(h, theta_max=D2R(60)):
    init = 1.0
    # why cannot calculate directly but use for loop??
    focal_in_tan = np.ceil(h/(2*np.math.tan(theta_max)))
    
    while theta_max < np.math.atan2(h/(2*init), 1):
        init = init + 1
    return init

# get the pinhole camera model including the focal and principal point to calculate the instinsics matrix
def get_pinhole_model(pin_w, pin_h):
    focal = focal_from_pinhole_intrinsics(pin_h, D2R(35))
    principal_point_x = pin_w / 2
    principal_point_y = pin_h / 2
    pinhole_model = PinholeIntrinsic(
        pin_w, pin_h, focal, principal_point_x, principal_point_y)
    return pinhole_model

# get specific degree rotation
def rotation_around_x(theta):
    # R = np.zeros((3, 3), dtype=np.float)
    # print(R)
    R = np.array([[1, 0, 0],
                  [0, np.math.cos(theta), -np.math.sin(theta)],
                  [0, np.math.sin(theta), np.math.cos(theta)]
                  ])
    #print(R)
    return R

# get specific degree rotation
def rotation_around_y(theta):
    R = np.zeros((3, 3), dtype=np.float)
    print(R)
    R = np.array([[np.math.cos(theta), 0, np.math.sin(theta)],
                  [0, 1, 0],
                  [-np.math.sin(theta), 0, np.math.cos(theta)]])
    print(R)
    return R

# get the extinct parameters of rotation matrix
def get_cubic_rotations():
    return [
        rotation_around_x(D2R(90)),  # up
        rotation_around_x(D2R(-90)),  # down
        rotation_around_y(D2R(-30)),  # front
        rotation_around_y(D2R(-90)), # front 1
        rotation_around_y(D2R(-150)),  # right
        rotation_around_y(D2R(-210)),  # behind
        rotation_around_y(D2R(-270)),  # left
        rotation_around_y(D2R(-330)), # front 1
    ]


def split2cubic(image):
    
    # obtain the shape of image
    img_w = image.shape[1]
    img_h = image.shape[0]
    
    # define the shape of targe
    pin_w = 512
    pin_h = 512
    
    
    # define sphere model and pinhold camera model
    sphere_model = SphericalIntrinsic(img_w, img_h)
    pinhole_model = get_pinhole_model(pin_w, pin_h)

    # calculate the rotation matrix and unset the pin_image matrix
    rot_matrix = get_cubic_rotations()
    pin_image = []


    # define agrid and set the bearing of pinhole camera
    for i in range(len(rot_matrix)):
        u = np.array(range(pin_w))
        v = np.array(range(pin_h))

        grid = np.array(np.meshgrid(u, v))
        print(grid)
        grid = np.transpose(grid, axes=(1, 2, 0)).reshape(-1, 2)
        print(grid)
        
        # left multiple the rotation matrix after pinhole camera model reproject he grid
        # is the same as rotation the grid after reproject the grid matrix in 3d sphere
        pinhole_bearing = np.matmul(
            np.linalg.inv(rot_matrix[i]), (pinhole_model.reproject(grid)))
        pinhole_bearing = pinhole_bearing.transpose((1, 0))
        
        # reshape the sphere in camera rotation
        sphere_uv = sphere_model.project(pinhole_bearing)
        
        map_x = sphere_uv[:, 0].reshape(pin_h, pin_w)
        map_y = sphere_uv[:, 1].reshape(pin_h, pin_w)

        temp = cv2.remap(image, map_x, map_y,
                         interpolation=cv2.INTER_CUBIC)

        pin_image.append(temp)

    return pin_image

def cubic2split(image):
    # obtain the shape of image
    img_w = image.shape[1]
    img_h = image.shape[0]
    
    # define the shape of targe
    pin_w = 512
    pin_h = 512
    
    
    # define sphere model and pinhold camera model
    sphere_model = SphericalIntrinsic(img_w, img_h)
    pinhole_model = get_pinhole_model(pin_w, pin_h)

    # calculate the rotation matrix and unset the pin_image matrix
    rot_matrix = get_cubic_rotations()
    pin_image = []


    # define agrid and set the bearing of pinhole camera
    for i in range(len(rot_matrix)):
        u = np.array(range(pin_w))
        v = np.array(range(pin_h))

        grid = np.array(np.meshgrid(u, v))
        print(grid)
        grid = np.transpose(grid, axes=(1, 2, 0)).reshape(-1, 2)
        print(grid)
        
        # left multiple the rotation matrix after pinhole camera model reproject he grid
        # is the same as rotation the grid after reproject the grid matrix in 3d sphere
        pinhole_bearing = np.matmul(
            np.linalg.inv(rot_matrix[i]), (pinhole_model.reproject(grid)))
        pinhole_bearing = pinhole_bearing.transpose((1, 0))
        
        # reshape the sphere in camera rotation
        sphere_uv = sphere_model.project(pinhole_bearing)
        
        map_x = sphere_uv[:, 0].reshape(pin_h, pin_w)
        map_y = sphere_uv[:, 1].reshape(pin_h, pin_w)

        temp = cv2.remap(image, map_x, map_y,
                         interpolation=cv2.INTER_CUBIC)

        pin_image.append(temp)

    return pin_image

# have not use this module in this project
def convert_kpts_back(imgshape, kpts_0, kpts_1, id_0, id_1):
    img_h = imgshape[0]
    img_w = imgshape[1]
    pin_w = 640
    pin_h = 480
    # pin_w = 320
    # pin_h = 240
    sphere_model = SphericalIntrinsic(img_w, img_h)
    pinhole_model = get_pinhole_model(pin_w, pin_h)

    rot_matrix = get_cubic_rotations()

    
    pinhole_bearing = np.matmul(
        np.linalg.inv(rot_matrix[id_0]), (pinhole_model.reproject(kpts_0)))
    pinhole_bearing = pinhole_bearing.transpose((1, 0))
    kpts_0 = sphere_model.project(pinhole_bearing)

    pinhole_bearing = np.matmul(
        np.linalg.inv(rot_matrix[id_1]), (pinhole_model.reproject(kpts_1)))
    pinhole_bearing = pinhole_bearing.transpose((1, 0))
    kpts_1 = sphere_model.project(pinhole_bearing)

    return kpts_0, kpts_1


def s2c():
    # image = cv2.imread(Path)
    # pin_image = split2cubic(image)
    # for _, img in enumerate(pin_image):
    #     cv2.imwrite(Path + str(_)+".jpg", img)
    ## get the main folder loacation and png in it as a list
    
    main_folder_path = "/home/ceri/4dage_ceri_program/Specular_Reflector_Filtering/panorama2normal/data/"                     
    png_name = os.listdir(main_folder_path)
    global_step = 0
    
    for each_png in png_name:
        if each_png.endswith(".jpg"):
            
            png_path = os.path.join(main_folder_path,each_png)
            image = cv2.imread(png_path)
            pin_image = split2cubic(image)
            
            save_folder_path = os.path.join(main_folder_path,str(global_step//125))
            
            if not os.path.exists(save_folder_path):
                os.mkdir(save_folder_path)
            
            for _, img in enumerate(pin_image):
                print(_)
                print(img)
                cv2.imwrite(save_folder_path + "/" + str(_+global_step*8)+".jpg", img)
            
            global_step = global_step + 1

def c2s():
    # image = cv2.imread(Path)
    # pin_image = split2cubic(image)
    # for _, img in enumerate(pin_image):
    #     cv2.imwrite(Path + str(_)+".jpg", img)
    ## get the main folder loacation and png in it as a list
    
    main_folder_path = "/home/ceri/4dage_ceri_program/Specular_Reflector_Filtering/panorama2normal/data/"                     
    png_name = os.listdir(main_folder_path)
    global_step = 0
    
    for each_png in png_name:
        if each_png.endswith(".jpg"):
            
            png_path = os.path.join(main_folder_path,each_png)
            image = cv2.imread(png_path)
            pin_image = split2cubic(image)
            
            save_folder_path = os.path.join(main_folder_path,str(global_step//125))
            
            if not os.path.exists(save_folder_path):
                os.mkdir(save_folder_path)
            
            for _, img in enumerate(pin_image):
                print(_)
                print(img)
                cv2.imwrite(save_folder_path + "/" + str(_+global_step*8)+".jpg", img)
            
            global_step = global_step + 1


if __name__ == '__main__':
    s2c()
    c2s()