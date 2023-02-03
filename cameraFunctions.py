"""Functions for DepthController by Deniz Namlısesli"""
import os
import numpy
import numpy as np
import math
from PIL import Image
import matplotlib.pyplot as plt
import open3d as o3d
import json
import cv2 as cv
import glob
import random
import pandas as pd
import scipy.integrate as integrate
from scipy import signal

class camera_funcitons:
    def __init__(self, bucket):
        self.bucket = bucket

    def mapper_func(self, depth_map_as_numpy_array):
        """Helper function for undistorted_image mapping depth_map mapped grayscale values 0-1000cm to 0-255"""
        count_i, count_j = depth_map_as_numpy_array.shape
        buffer_array = np.random.random(size=depth_map_as_numpy_array.shape)
        for i in range(count_i):
            for j in range(count_j):
                buffer_array[i][j] = ((depth_map_as_numpy_array[i][j])*255/1000)

        return buffer_array

    def inverse_mapper_func(self, depth_map_as_numpy_array, undistorted_image):
        """Helper function for undistorted_image mapping depth_map mapped grayscale values 0-255 to 0-1000cm"""
        image_array = np.asarray(undistorted_image)
        count_i,count_j = image_array.shape
        inverse_buffer_array = np.random.random(size=depth_map_as_numpy_array.shape)
        """There is slight error comming from mapping and inverse mapping"""
        for i in range(count_i):
            for j in range(count_j):
                inverse_buffer_array[i][j] = ((image_array[i][j])*1000/255)

        return inverse_buffer_array

    def undistored_image(self, depth_map_as_numpy_array):
        """Applying undistortion to the given depth_map by mapping its values between 0-255 for grayscale representation
        and applying undistortion from cv2 library with the intrinsic matrix and distortion coefficients provided from
        camera calibration data created by MATLAB and exported with -camera parameters to CV2-"""
        mtx = [[432.0252630385620, 0, 320.3227215467547],
               [0, 433.5203652740541, 240.1869577174981],
               [0, 0, 1.0000]]
        dist = [0.077441793222239, -0.082399364511784, 0, 0, 0]
        mtx = np.array(mtx)
        dist = np.array(dist)

        h, w = (480, 640)
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

        """Applying undistortion to mapped 0-255 array as if its a grayscaled image"""
        img_array = self.mapper_func(depth_map_as_numpy_array)
        dst = cv.undistort(img_array, mtx, dist, None, newcameramtx)
        x, y, w, h = roi
        dst = dst[y:y + h, x:x + w]

        """By inverse mapping the undistored cropped image using grayscaled values we obtain distance info
        by some margin of error"""
        undistorted_array = self.inverse_mapper_func(depth_map_as_numpy_array, dst)
        return undistorted_array

    def return_example_depth_data_nparray_from_firebase(self, date_of_interest, hh_mm_ss):
        """Downloading image and depth data for testing new data in case its requested"""
        example_blob = self.bucket.blob(f"images/depthdata/{date_of_interest}/{hh_mm_ss}.json")
        depthJSON = json.loads(example_blob.download_as_text())
        myArray = depthJSON["depth_data"]
        depth_map = np.array(myArray)
        depth_map = np.fliplr(depth_map)
        print("Example Array Downloaded Successfully")
        return depth_map

    def return_point_cloud_ready_nparray(self, depth_map, depth_list_element_name=None):
        """When presented with unprocessed depth_map (480x640) or (640x480) (IDK which is applied here)
        returns (307200, 3) shaped point_cloud_ready_depth_map

        depth_list_element is an optional value if its defined then return_bright_spot function is avaliable.

        If return_bright_spot will used then QRPfRA_image_dataset must be downloaded for reach.
        """
        if depth_list_element_name==None:
            count_i, count_j = depth_map.shape
            print("return_point_cloud_ready_nparray: depth_map.shape=", depth_map.shape)
            array_list = []
            for i in range(count_i):
                for j in range(count_j):
                    array_list.append([i, j, depth_map[i][j]])
            point_cloud_ready_depth_map = np.array(array_list)

            return point_cloud_ready_depth_map
        elif depth_list_element_name!=None:
            count_i, count_j = depth_map.shape
            print("return_point_cloud_ready_nparray: depth_map.shape=", depth_map.shape)
            array_list = []
            bright_spot_list = self.return_bright_spots(depth_list_element_name,threshold_min=230)
            for i in range(count_i):
                for j in range(count_j):
                    if [i,j] in bright_spot_list:
                        ###print("Bright spot eliminated")
                        array_list.append([i, j, 0])
                    else:
                        array_list.append([i, j, depth_map[i][j]])
            point_cloud_ready_depth_map = np.array(array_list)
            return point_cloud_ready_depth_map



    def download_jpg_or_depth_to_dir(self, date_of_interest, save_dir, download_photo = True, download_depth = False):
        """Downloading image and depth files as individual datasets to stated directory in this case its desktop.
        IMPORTANT NOTE: Downloads only image and sensory files otherwise set depth dataset as True. For each download
        request sensory information is downloaded.(its bundled with other sets)

        FUTURE FEATURE IDEA: Adding session counter for firebase and updating firebase related functions for sessions
        accordingly.
        """
        blob_name_list = [blob.name for blob in self.bucket.list_blobs()]
        if download_photo and (not download_depth):
            find_image_blobs = []
            date_filter = f"photodata/{date_of_interest}"
            for i in blob_name_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[0]:
                    find_image_blobs.append(i)

            img_dataset_dir = "QRPfRA_image_dataset"
            download_dir_path = os.path.join(save_dir, img_dataset_dir)

            if os.path.exists(download_dir_path) == False:
                os.makedirs(download_dir_path)
            else:
                pass

            for i in find_image_blobs:
                image = self.bucket.blob(i)
                file_name = download_dir_path + f"/{os.path.split(i)[1]}".replace(".heif", ".png")
                image.download_to_filename(file_name)
                print("Downloading...." + file_name)
                """PIL library could not read the .heif format just by it self (without using ext lib)
                but it can read the extension changed verison (.png) of the .heif file. But again MATLAB
                cannot read extension changed version eventhough PIL can read it.
                   So I convert the extension to .png and save the .png photo as .jpg using PIL which is
                readable from MATLAB"""
                temp_image = Image.open(file_name)
                temp_image.save((download_dir_path + f"/{os.path.split(i)[1]}".replace(".heif", ".jpg")), "JPEG",
                                quality=100, optimize=True, progressive=True)
                print("Deletinng old png file")
                os.remove(file_name)
                print("Done")

            ##Downloading Sensory Information
            find_sensor_blobs = []
            date_filter = f"sensordata/{date_of_interest}"
            for i in blob_name_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[0]:
                    find_sensor_blobs.append(i)

            sensor_dataset_dir = "QRPfRA_sensor_dataset"
            download_dir_path = os.path.join(save_dir, sensor_dataset_dir)

            if os.path.exists(download_dir_path) == False:
                os.makedirs(download_dir_path)
            else:
                pass

            for i in find_sensor_blobs:
                print("Downloading sensor data\n")
                sensor = json.loads(self.bucket.blob(i).download_as_text())
                file_name = download_dir_path + f"/{os.path.split(i)[1]}"

                with open(file_name, 'w') as outfile:
                    json.dump(sensor, outfile)
                print("Done")
        ##Download Depth and Sensor Data
        elif download_depth and (not download_photo):
            find_depth_blobs = []
            date_filter = f"depthdata/{date_of_interest}"
            for i in blob_name_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[0]:
                    find_depth_blobs.append(i)

            depth_dataset_dir = "QRPfRA_depth_dataset"
            download_dir_path = os.path.join(save_dir, depth_dataset_dir)

            if os.path.exists(download_dir_path) == False:
                os.makedirs(download_dir_path)
            else:
                pass

            for i in find_depth_blobs:
                print("Downloading depth dat\n")
                depth = json.loads(self.bucket.blob(i).download_as_text())
                file_name = download_dir_path + f"/{os.path.split(i)[1]}"

                with open(file_name, 'w') as outfile:
                    json.dump(depth, outfile)
                print("Done")

            ##Downloading Sensory Information
            find_sensor_blobs = []
            date_filter = f"sensordata/{date_of_interest}"
            for i in blob_name_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[0]:
                    find_sensor_blobs.append(i)

            sensor_dataset_dir = "QRPfRA_sensor_dataset"
            download_dir_path = os.path.join(save_dir, sensor_dataset_dir)

            if os.path.exists(download_dir_path) == False:
                os.makedirs(download_dir_path)
            else:
                pass

            for i in find_sensor_blobs:
                print("Downloading sensor data\n")
                sensor = json.loads(self.bucket.blob(i).download_as_text())
                file_name = download_dir_path + f"/{os.path.split(i)[1]}"

                with open(file_name, 'w') as outfile:
                    json.dump(sensor, outfile)
                print("Done")

        elif (download_depth and download_photo):
            find_image_blobs = []
            date_filter = f"photodata/{date_of_interest}"
            for i in blob_name_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[0]:
                    find_image_blobs.append(i)

            img_dataset_dir = "QRPfRA_image_dataset"
            download_dir_path = os.path.join(save_dir, img_dataset_dir)

            if os.path.exists(download_dir_path) == False:
                os.makedirs(download_dir_path)
            else:
                pass

            for i in find_image_blobs:
                image = self.bucket.blob(i)
                file_name = download_dir_path + f"/{os.path.split(i)[1]}".replace(".heif", ".png")
                image.download_to_filename(file_name)
                print("Downloading...." + file_name)
                """PIL library could not read the .heif format just by it self (without using ext lib)
                but it can read the extension changed verison (.png) of the .heif file. But again MATLAB
                cannot read extension changed version eventhough PIL can read it.
                   So I convert the extension to .png and save the .png photo as .jpg using PIL which is
                readable from MATLAB"""
                temp_image = Image.open(file_name)
                temp_image.save((download_dir_path + f"/{os.path.split(i)[1]}".replace(".heif", ".jpg")), "JPEG",
                                quality=100, optimize=True, progressive=True)
                print("Deletinng old png file")
                os.remove(file_name)
                print("Done")

            find_depth_blobs = []
            date_filter = f"depthdata/{date_of_interest}"
            for i in blob_name_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[0]:
                    find_depth_blobs.append(i)

            depth_dataset_dir = "QRPfRA_depth_dataset"
            download_dir_path = os.path.join(save_dir, depth_dataset_dir)

            if os.path.exists(download_dir_path) == False:
                os.makedirs(download_dir_path)
            else:
                pass

            for i in find_depth_blobs:
                print("Downloading depth dat\n")
                depth = json.loads(self.bucket.blob(i).download_as_text())
                file_name = download_dir_path + f"/{os.path.split(i)[1]}"

                with open(file_name, 'w') as outfile:
                    json.dump(depth, outfile)
                print("Done")

            ##Downloading Sensory Information
            find_sensor_blobs = []
            date_filter = f"sensordata/{date_of_interest}"
            for i in blob_name_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[0]:
                    find_sensor_blobs.append(i)

            sensor_dataset_dir = "QRPfRA_sensor_dataset"
            download_dir_path = os.path.join(save_dir, sensor_dataset_dir)

            if os.path.exists(download_dir_path) == False:
                os.makedirs(download_dir_path)
            else:
                pass

            for i in find_sensor_blobs:
                print("Downloading sensor data\n")
                sensor = json.loads(self.bucket.blob(i).download_as_text())
                file_name = download_dir_path + f"/{os.path.split(i)[1]}"

                with open(file_name, 'w') as outfile:
                    json.dump(sensor, outfile)
                print("Done")
        else:
            exit(".............Not taken any action, you have bugs to fix.............")

        image_dataset_dir = "QRPfRA_image_dataset"; image_dataset_dir = os.path.join(save_dir,image_dataset_dir)
        depth_dataset_dir = "QRPfRA_depth_dataset"; depth_dataset_dir = os.path.join(save_dir, depth_dataset_dir)
        sensor_dataset_dir = "QRPfRA_sensor_dataset"; sensor_dataset_dir = os.path.join(save_dir, sensor_dataset_dir)
        return image_dataset_dir, depth_dataset_dir, sensor_dataset_dir

    def list_of_image_and_depth_files(self, image_dir, depth_dir, sensor_dir):
        """Returns list of image and depth files for given respected directories. Useful for returning random or
        selected test file."""
        image_extension = "*.jpg"
        depth_extension = "*.json"
        sensor_extension = "*.json"
        image_full_path = os.path.join(image_dir, image_extension)
        depth_full_path = os.path.join(depth_dir, depth_extension)
        sensor_full_path = os.path.join(sensor_dir, sensor_extension)
        image_file_list = glob.glob(image_full_path)
        depth_file_list = glob.glob(depth_full_path)
        sensor_file_list = glob.glob(sensor_full_path)
        return image_file_list, depth_file_list, sensor_file_list

    def return_example_depth_data_nparray_from_existing_file(self, depth_file_list, randomized=True, hh_mm_ss=None):
        """For offline and testing usage returns an unprocessed random depth_map from list of files stated otherwise."""
        if (randomized == True):
            depthJSON = json.load(open(depth_file_list[random.randint(0, len(depth_file_list)-1)], 'r'))
            myArray = depthJSON["depth_data"]
            depth_map = np.array(myArray)
            depth_map = np.fliplr(depth_map)
            return depth_map
        elif (randomized == False):
            date_filter = f"{hh_mm_ss}.json"
            find_depth_blobs = []
            for i in depth_file_list:
                head_tail = os.path.split(i)
                if date_filter in head_tail[1]:
                    find_depth_blobs.append(i)
            if (find_depth_blobs == []):
                exit("Can't find a json file with specified hh_mm_ss. Check input")
            elif (int(len(find_depth_blobs)) == 1):
                depthJSON = json.load(open(find_depth_blobs[0], 'r'))
                myArray = depthJSON["depth_data"]
                depth_map = np.array(myArray)
                depth_map = np.fliplr(depth_map)
                return depth_map
            else:
                exit("There are more than one item with the same name in the list.")
        else:
            exit("Can't return anything, there is bugs to fix")

    def plot_and_show_3d_geometry(self, example_depth_map, point_ready_depth_map, show3D=False):
        """--------------------PLOTTING ORIGINAL DEPTH_MAP FOR CONTROL //\\ PLOTTING POINT_CLOUD_INFO-------------------"""
        if show3D==True:
            plt.imshow(np.fliplr(np.flipud(example_depth_map.transpose())))
            plt.show()
            points = point_ready_depth_map
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            o3d.visualization.draw_geometries([pcd])
        else:
            plt.imshow(np.fliplr(np.flipud(example_depth_map.transpose())))
            plt.show()
        return None

    def return_bright_spots(self, image_hh_mm_ss, threshold_min=240, threshold_max=255, img_dir="/Users/deniz/Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_image_dataset"):
        """QRPfRA_image_dataset must be downloaded in order to use this function"""
        image_hh_mm_ss = image_hh_mm_ss + ".jpg"
        im_path = os.path.join(img_dir, image_hh_mm_ss)
        image = cv.imread(im_path)
        gray_scaled = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        gray_im = cv.medianBlur(gray_scaled, 5)
        ret, thresh_img = cv.threshold(gray_im, threshold_min, threshold_max, cv.THRESH_BINARY)
        im_as_arr = np.flipud(np.asarray(thresh_img).transpose())
        #plt.imshow(im_as_arr)
        #plt.show()
        count_i, count_j = im_as_arr.shape
        bright_px_ls = []
        for i in range(count_i):
            for j in range(count_j):
                if im_as_arr[i, j] == 255:
                    """This is appended as i,j corresponds to y,x in terms of cartesian system while x is horizontal and
                    y is vertical."""
                    brigt_px = [i, j]
                    bright_px_ls.append(brigt_px)
        return bright_px_ls

    def clearDataBase(self):
        """Use this in case of different tree naming with mobile app.
        Some permission error about firebase."""
        self.bucket.delete_blobs("images/photodata")
        return None

    def clearDatabase_for_given_date(self, date):
        """date as = dd-mm-yy, for example 21-08-22
        Some permission error about firebase."""
        blob_name_list = [blob.name for blob in self.bucket.list_blobs()]
        photo_filter = f"photodata/{date}"
        depth_filter = f"depthdata/{date}"
        sensor_filter = f"sensordata/{date}"
        for i in blob_name_list:
            head_tail = os.path.split(i)
            if (photo_filter or depth_filter or sensor_filter) in head_tail[0]:
                print(i)
                self.bucket.delete_blobs(i)
                print(f"Successfully deleted: {i}")

        return None

class matrix_operations:
    def __init__(self, np):
        self.np = np

    def spherical_to_cartesian_conversion(self, point_cloud_array, depth_filter=False, depth_min_cm=20, depth_max_cm=500):
        """Below code is converting every element of point_cloud_array with respect to it's pixel's location.
        Taking pixel location data with its depth value, mapping pixel location to angle and converting with conversion
        matrix. Assigning converted values to buffer array and return the converted buffer array with exact shape of
        point_cloud_array.

        IMPORTANT NOTE: Cos and Sin as in terms of radians"""
        count_i, count_j = point_cloud_array.shape
        print("spherical_to_cartesian_conversion: point_cloud_array.shape =",point_cloud_array.shape)
        buffer_cart_array = np.random.random(size=point_cloud_array.shape)
        for i in range(count_i):
            pixel_x, pixel_y, r_depth = point_cloud_array[i]
            tetha_as_rad, phi_as_rad = self.angle_pixel_maper(pixel_x, pixel_y, point_cloud_array[i][2])
            """adding radius to angle mapper with point_cloud_array[i][2] results stunning spheres"""
            spherical_matrix = [point_cloud_array[i][2], tetha_as_rad, phi_as_rad]
            spherical_matrix = np.array(spherical_matrix)
            created_matrix = self.conversion_matrix(spherical_matrix, tetha_as_rad, phi_as_rad)
            if depth_filter:
                if ((abs(point_cloud_array[i][2]) < depth_max_cm) and (depth_min_cm < abs(point_cloud_array[i][2]))):
                    """np.array([-created_matrix[0], created_matrix[1], -created_matrix[2]])
                    Also can be added to solidify looking but I can also normalize it on matlab."""
                    x,y,z = (created_matrix[0], created_matrix[1], created_matrix[2])
                    buffer_cart_array[i] = np.array([x*1000, y, z*1000])#created_matrix
                else:
                    created_matrix[2]
                    continue
            elif (depth_filter == False):
                buffer_cart_array[i] = created_matrix
            else:
                exit("Can't convert to cartesian check function parameters")

        return buffer_cart_array

    def conversion_matrix(self, input_matrix ,tetha_as_rad, phi_as_rad):
        """This is the spherical to cartesian coordinate system conversion matrix. Returns respected cartesian matrix."""
        tetha = tetha_as_rad
        phi = phi_as_rad
        conversion_matrix = [[(np.sin(tetha)*np.cos(phi)), (np.cos(tetha)*np.cos(phi)), (-np.sin(phi))],
                             [(np.sin(tetha)*np.sin(phi)), np.cos(tetha)*np.sin(phi), (np.cos(phi))],
                             [np.cos(tetha), (-np.sin(tetha)), 0]]
        conversion_matrix = np.array(conversion_matrix)
        '''Transpose of the matrix because I write the other one'''
        conversion_matrix = conversion_matrix.transpose()

        cartesian_matrix = np.matmul(conversion_matrix, input_matrix)
        return cartesian_matrix

    def point_conversion(self, point_cloud_array):
        """Even though this shares the same philosophy with matrix conversion, point_conversion can't reproduce the same
         output.
         NOTE: I need to look at this some time in the future and investigate why I can't get the same output."""
        count_i, count_j = point_cloud_array.shape
        print(point_cloud_array.shape)
        buffer_cart_array = np.random.random(size=point_cloud_array.shape)
        for i in range(count_i):
            pixel_x, pixel_y, r_depth = point_cloud_array[i]
            tetha_as_rad, phi_as_rad = self.angle_pixel_maper(pixel_x, pixel_y, r_depth)
            pc_x = r_depth*np.sin(tetha_as_rad)*np.cos(phi_as_rad)
            pc_y = r_depth*np.sin(tetha_as_rad)*np.sin(phi_as_rad)
            pc_z = r_depth*np.cos(tetha_as_rad)
            cartesian_mtx = np.array([pc_x, pc_y, pc_z])
            buffer_cart_array[i] = cartesian_mtx

        return buffer_cart_array

    def angle_pixel_maper(self, pixel_x, pixel_y, radius):
        """Respected angle values are for angle phi 36.87, for angle tetha 23.07 or (possibly) 16.26
        IMPORTANT NOTE: These camera angles need to be measured again for better accuracy."""
        row, col = (640, 480)
        imc_y, imc_x = (321, 241)#(int(row / 2), int(col / 2))
        foc_y, foc_x = (432, 433)
        """Take a good measurement of the camera parameters"""
        top_to_center = 0.03687 #This values are achived with good measurement np.deg2rad(36.87)#
        right_to_center = 0.02467 #This values are achived with good measurement np.deg2rad(24.67)#
        tetha_as_deg = ((pixel_x-imc_x)*right_to_center/imc_x)
        phi_as_deg = ((pixel_y-imc_y)*top_to_center/imc_y)
        """tetha_as_deg = (pixel_x-imc_x)*np.cos(np.deg2rad(top_to_center))*np.sin(np.deg2rad(right_to_center))/math.pi
        phi_as_deg = (pixel_y-imc_y)*np.sin(np.deg2rad(right_to_center))*np.cos(np.deg2rad(top_to_center))/math.pi
        tetha_as_deg = np.arcsin((((np.sin(right_to_center))/imc_x)*(imc_x-pixel_x))) #np.arccos(1-(((np.sin(top_to_center))/top_to_center)*pixel_x))
        phi_as_deg = np.arcsin((((np.sin(top_to_center))/imc_y)*(imc_y-pixel_y)))#np.arccos(1-(((np.sin(right_to_center))/right_to_center)*pixel_y))"""
        """
        this pne makes cool 3D plots when parameters changed
        tetha_as_deg = abs(top_to_center*math.exp(0.0028*(pixel_x-imc_x)))
        phi_as_deg = abs(right_to_center*math.exp(0.0028*(pixel_y-imc_y)))
        
        if pixel_y<400:
            phi_as_deg = (pixel_y - imc_y) * 40.47 / imc_y
        else:
            phi_as_deg = (pixel_y - imc_y) * 32.30 / imc_y"""
        tetha_as_rad, phi_as_rad = (np.deg2rad(tetha_as_deg), np.deg2rad(phi_as_deg)) #tetha_as_deg,phi_as_deg#
        return tetha_as_rad, phi_as_rad


class point_cloud_functions:
    def __init__(self, np, camera_func, matrix_op):
        self.np = np
        self.camera_func = camera_func
        self.matrix_op = matrix_op

    def find_min_x_and_y(self, converted_cloud_array):
        """Find the most negative values for given array"""
        count_i, count_j = converted_cloud_array.shape
        min_x = 0
        min_y = 0
        min_z = 0
        for i in range(count_i):
            if(converted_cloud_array[i][0] < min_x):
                min_x = converted_cloud_array[i][0]

            if(converted_cloud_array[i][1] < min_y):
                min_y = converted_cloud_array[i][1]

            if(converted_cloud_array[i][2] < min_z):
                min_z = converted_cloud_array[i][2]
        return min_x, min_y, min_z

    def normalized_cloud_array(self, converted_cloud_array):
        """Using most negative values to create same array with all positive values for applying transformations
        to it"""
        min_x, min_y, min_z = self.find_min_x_and_y(converted_cloud_array)
        min_array = [min_x, min_y, min_z]
        count_i, count_j = converted_cloud_array.shape
        buffer_cart_array = np.random.random(size=converted_cloud_array.shape)
        for i in range(count_i):
            for j in range(count_j):
                buffer_cart_array[i][j] = abs(min_array[j]) + converted_cloud_array[i][j]

        return buffer_cart_array

    def save_point_cloud_to_csv(self, point_cloud_array, save_to, floating_point_precision=2, name=None):
        if (name != None):
            pcFileName = f"{name}.csv"
            save_path = os.path.join(save_to, pcFileName)
            if os.path.exists(save_path) == False:
                np.savetxt(f'{save_path}', point_cloud_array, fmt=f'%.{floating_point_precision}f', delimiter=",")
                print(f"Point Cloud Array saved to {save_path}")
            else:
                NameError
                exit(f"Can't save the file: {save_path}, there is a existing file with the same name")

        elif (name == None):
            version = 0
            pcFileName = f"point_cloud_array_({version}).csv"
            save_path = os.path.join(save_to, pcFileName)
            while os.path.exists(save_path):
                version += 1
                pcFileName = f"point_cloud_array_({version}).csv"
                save_path = os.path.join(save_to, pcFileName)
                if not os.path.exists(save_path):
                    print(save_path)
                    break
                else:
                    continue
            if os.path.exists(save_path) == False:
                np.savetxt(f'{save_path}', point_cloud_array, fmt=f'%.{floating_point_precision}f', delimiter=",")
                print(f"Point Cloud Array saved to {save_path}")
            else:
                exit(f"Can't save the file: {save_path}")

        else:
            exit("Great Error Comes With Great Responsibility")
        return None

    def color_array_for_point_cloud_to_csv(self):
        """To get exact mapping I must transform or map the rgb values to converted depth map

        TO-DO: Because this is more cosmetic than necessity I can finish this up later in the project.
           -> Convert the pixels to spherical accordingly.
        """
        """TODO: Apply pixel mapping to photo values to determine which pixel has respected RGB or possibly BGR values
        Add directory input to the function."""
        image_dataset_dir = "/Users/deniz/Desktop/QRPfRA_image_dataset/"
        image_extension = "*.jpg"
        full_image_path = os.path.join(image_dataset_dir, image_extension)
        image_list = glob.glob(full_image_path)
        for i in image_list:
            img = Image.open(i)
            img_array = np.asarray(img).astype(int)
            count_i, count_j, rgb_mtx = img_array.shape
            array_array = np.random.random((307200,3))
            cnt = 0
            for count_i in range(count_i):
                for count_j in range(count_j):
                    array_array[cnt] = img_array[count_i][count_j]
                    cnt += 1

            colored_points_array = array_array#np.array(array_list)
            saving_dir = "/Users/deniz/Desktop/QRPfRA_colorPoints/"
            saving_path = f"{(os.path.split(i)[1]).split('.')[0]}.csv"
            path = os.path.join(saving_dir, saving_path)
            if os.path.exists(saving_dir):
                np.savetxt(path, colored_points_array, delimiter=',')
                print(f"File saved to {path}")
            else:
                os.makedirs(saving_dir)
                np.savetxt(path, colored_points_array, delimiter=',')
                print(f"Directory {saving_dir} created and file is saved to {path}.")

    def save_point_cloud_to_dir_as_csv_with_exact_naming(self, depth_list, save_dir, depth_filter=False, depth_max_cm=500,
                                                         depth_min_cm=20, floating_point_precision=2, delete_bright_spots=False):
        for i in depth_list:
            file_name = ((os.path.split(i)[1]).split('.')[0])
            depthJSON = json.load(open(i, 'r'))
            myArray = depthJSON["depth_data"]
            depth_map = np.array(myArray)
            depth_map = np.fliplr(depth_map)
            if delete_bright_spots==True:
                point_ready_depth_map = self.camera_func.return_point_cloud_ready_nparray(depth_map=depth_map,
                                                                                     depth_list_element_name=file_name)
            else:
                point_ready_depth_map = self.camera_func.return_point_cloud_ready_nparray(depth_map=depth_map)
            converted_point_cloud = self.matrix_op.spherical_to_cartesian_conversion(point_cloud_array=point_ready_depth_map,
                                                                                        depth_filter=depth_filter,
                                                                                        depth_max_cm=depth_max_cm,
                                                                                        depth_min_cm=depth_min_cm)
            saving_dir = f"{save_dir}/QRPfRA_pointClouds/"
            saving_path = f"{file_name}.csv"
            path = os.path.join(saving_dir, saving_path)
            if os.path.exists(saving_dir):
                np.savetxt(path, converted_point_cloud, delimiter=',', fmt=f'%.{floating_point_precision}f')
                print(f"Point Cloud Array saved to {path}")
            else:
                os.makedirs(saving_dir)
                np.savetxt(path, converted_point_cloud, delimiter=',', fmt=f'%.{floating_point_precision}f')
                print(f"Point Cloud Array saved to {path}")

        return None


#Additional useless transform function
    """    def transform_normalized_cloud_array(self, converted_cloud_array):
            normalized_arr = self.normalized_cloud_array(converted_cloud_array)
            count_i, count_j = normalized_arr.shape
            buffer_cart_array = np.random.random(size=normalized_arr.shape)
            for i in range(count_i):
                bff_arr = [normalized_arr[i][0], normalized_arr[i][1], -normalized_arr[i][2]]
                buffer_cart_array[i] = np.array(bff_arr)

            return buffer_cart_array"""

class sensor_preprocessing:
    def __init__(self, bucket):
        self.bucket = bucket

    def read_and_return_pd(self,file_path= None, from_firebase = False, date_of_interest=None, hh_mm_ss=None):
        roll = []; pitch = []; yaw = []; rotationRateX = []; rotationRateY = []; rotationRateZ = []
        userAccelerationX = []; userAccelerationY = []; userAccelerationZ = []; quaternionX = []
        quaternionY = []; quaternionZ = []; quaternionW = []; magneticX = []; magneticY = []; magneticZ = []
        normalizedAccX = []; normalizedAccY = []; normalizedAccZ = []
        general_list = [roll, pitch, yaw, rotationRateX, rotationRateY, rotationRateZ, userAccelerationX,
                        userAccelerationY, userAccelerationZ, quaternionX, quaternionY, quaternionZ, quaternionW, magneticX,
                        magneticY, magneticZ, normalizedAccX, normalizedAccY, normalizedAccZ]
        sensor_list = self.load_sensor_json_list(file_path=file_path, from_firebase=from_firebase,
                                                 date_of_interest=date_of_interest, hh_mm_ss=hh_mm_ss)
        for line in sensor_list:
            temp_list = line.split(',')
            for i in range(len(temp_list)):
                general_list[i].append(float(temp_list[i]))

        dataframe = pd.DataFrame(list(zip(roll, pitch, yaw, rotationRateX, rotationRateY, rotationRateZ, userAccelerationX,
                        userAccelerationY, userAccelerationZ, quaternionX, quaternionY, quaternionZ, quaternionW, magneticX,
                                          magneticY, magneticZ, normalizedAccX, normalizedAccY, normalizedAccZ)), columns=[
                                                        "roll", "pitch", "yaw", "rotationRateX", "rotationRateY",
                                                        "rotationRateZ", "userAccelerationX",
                                                        "userAccelerationY", "userAccelerationZ", "quaternionX",
                                                        "quaternionY", "quaternionZ", "quaternionW", "magneticX", "magneticY",
                                                        "magneticZ", "normalizedAccX", "normalizedAccY", "normalizedAccZ"])

        return dataframe

    def load_sensor_json_list(self, file_path = None, from_firebase = False, date_of_interest=None, hh_mm_ss=None):
        if (from_firebase and (file_path == None)):
            example_blob = self.bucket.blob(f"images/sensordata/{date_of_interest}/{hh_mm_ss}-10.json")
            sensorJSON = json.loads(example_blob.download_as_text())
            print("Sensory Data Downloaded Successfully")
            myArray = sensorJSON["spatial_data"]
            #myArray.pop(0)
            if myArray != ValueError:
                sensor_list = np.array(myArray)
                return sensor_list
            else:
                pass

        elif ((file_path != None) and (from_firebase == False)):
            file = open(file_path, 'r')
            sensorJSON = json.load(file)
            print("Sensory Data Opened Successfully")
            myArray = sensorJSON["spatial_data"]
            #myArray.pop(0)
            ##sensor_list = np.array(myArray)
            ##return sensor_list
            if myArray != ValueError:
                sensor_list = np.array(myArray)
                return sensor_list
            else:
                pass
        else:
            exit("Can't open file nor downloaded from firebase.")

    def write_pd_to_csv(self, dataframe, saving_path, name=None):
        if (name != None):
            pcFileName = f"{name}.csv"
            save_path = os.path.join(saving_path, pcFileName)
            if os.path.exists(save_path) == False:
                dataframe.to_csv(save_path, index=False)
                print(f"Sensor dataframe saved to {save_path} as CSV")
            else:
                NameError
                exit(f"Can't save the file: {save_path}, there is a existing file with the same name")

        elif (name == None):
            version = 0
            pcFileName = f"sensor_data_({version}).csv"
            save_path = os.path.join(saving_path, pcFileName)
            while os.path.exists(save_path):
                version += 1
                pcFileName = f"sensor_data_({version}).csv"
                save_path = os.path.join(saving_path, pcFileName)
                if not os.path.exists(save_path):
                    print(save_path)
                    break
                else:
                    continue
            if os.path.exists(save_path) == False:
                dataframe.to_csv(save_path, index=False)
                print(f"Sensor dataframe saved to {save_path} as CSV")
            else:
                exit(f"Can't save the file: {save_path}")

        else:
            exit("Great Error Comes With Great Responsibility")
        return None

    def get_accel_displacement(self, dataframe, frequency=100):
        """Correcting phones coordinate system then return net displacement of phone in terms of meters"""
        df_x_array = np.array([value * 9.81 for value in dataframe["userAccelerationX"].values.tolist()])
        df_y_array = np.array([value * 9.81 for value in dataframe["userAccelerationY"].values.tolist()])
        df_z_array = np.array([value * 9.81 for value in dataframe["userAccelerationZ"].values.tolist()])
        pi = math.pi

        pitch, roll, yaw = "pitch", "roll", "yaw"
        x_arr = np.random.random(df_x_array.size)
        y_arr = np.random.random(df_x_array.size)
        z_arr = np.random.random(df_x_array.size)

        """for i in range(len(df_x_array)):
            '''Note to myself: ADD OTHER COMPONENTS OF THE VECTORS'''
            x_arr[i] = df_x_array[i] * np.cos(dataframe[yaw][i]) * np.sin(dataframe[roll][i])

            y_arr[i] = df_y_array[i] * np.cos(dataframe[pitch][i]) * np.sin(dataframe[yaw][i])

            z_arr[i] = df_z_array[i] * np.cos(dataframe[roll][i]) * np.sin(dataframe[pitch][i])"""

        """arrX = self.return_true_arrays(dataframe, df_x_array, 'x')
        arrY = self.return_true_arrays(dataframe, df_y_array, 'y')
        arrZ = self.return_true_arrays(dataframe, df_z_array, 'z')

        df_x_array, df_y_array, df_z_array = self.sum_axes(arrX, arrY, arrZ)"""

        """x_arr = self.return_true_arrays(dataframe, df_x_array, 'x')
        y_arr = self.return_true_arrays(dataframe, df_y_array, 'y')
        z_arr = self.return_true_arrays(dataframe, df_z_array, 'z')

        df_x_array, df_y_array, df_z_array = self.sum_axes(x_arr, y_arr, z_arr)"""

        deltaX = self.return_loc_change(df_x_array, frequency)
        deltaY = self.return_loc_change(df_y_array, frequency)
        deltaZ = self.return_loc_change(df_z_array, frequency)

        return deltaX ,deltaY ,deltaZ

    def return_loc_change(self, respected_numpy_array, frequency=100):
        """Reading respected array and applying necessary filters to it.
        Input: nparray, frequency (Hz)
        Output: Float (net location change)

          A Savitzky–Golay filter is a digital filter that can be applied to a set of digital data points for the purpose
        of smoothing the data, that is, to increase the precision of the data without distorting the signal tendency.
        This is achieved, in a process known as convolution, by fitting successive sub-sets of adjacent data points
        with a low-degree polynomial by the method of linear least squares.
         Source: https://en.wikipedia.org/wiki/Savitzky–Golay_filter

          signal.medfit: Perform a median filter on an N-dimensional array.
        Apply a median filter to the input array using a local window-size given by kernel_size.
        The array will automatically be zero-padded.
          Source: https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.medfilt.html
        """
        dt = np.linspace(0, len(respected_numpy_array)/frequency, len(respected_numpy_array))
        respected_numpy_array = signal.medfilt(respected_numpy_array, kernel_size=5)
        #plt.plot(respected_numpy_array)
        respected_numpy_array = signal.savgol_filter(respected_numpy_array, window_length=255, polyorder=3)
        #plt.plot(respected_numpy_array)

        velocity = integrate.cumtrapz(respected_numpy_array, dt, initial=0)
        velocity = signal.savgol_filter(velocity, window_length=255, polyorder=3)
        velocity = signal.detrend(velocity)

        #plt.plot(velocity)
        location_array = integrate.cumtrapz(velocity, dt)
        #plt.plot(location_array)
        #plt.show()

        net_loc_change = 0
        for i in range(len(location_array)):
            net_loc_change += location_array[i]

        return int(net_loc_change*10000)/100 ##Converting to centimeters with 1 decimal points

    def converted_matrix(self, dataframe, df_array, df_axis):
        """Input:
            dataframe: Pandas Dataframe
            df_axis: Axis of interest
        ....Not working....
        """
        if df_axis == 'x':
            tetha, phi = ("yaw", "roll")
        elif df_axis == 'y':
            tetha, phi = ("pitch", "yaw")
        elif df_axis == 'z':
            tetha, phi = ("roll", "pitch")
        else:
            exit('Input not recognized. Please input, x, y or z for axis')

        buffer_array = []#np.random.random(size=df_array.shape)
        tetha_ = np.array(dataframe[tetha])
        phi_ = np.array(dataframe[phi])
        pi = math.pi
        print(tetha)
        for i in range(len(df_array)-1):
            tetha = tetha_[i]
            tetha_1 = tetha_[i+1]
            phi = phi_[i]
            phi_1 = phi_[i+1]
            conversion_matrix = [[(np.sin(tetha_1 - tetha) * np.cos(phi_1 - phi)), (np.cos(tetha_1 -tetha) * np.cos(phi_1 - phi)), (-np.sin(phi_1 - phi))],
                                 [(np.sin(tetha_1 -tetha) * np.sin(phi_1- phi)), np.cos(tetha_1 -tetha) * np.sin(phi_1 - phi), (np.cos(phi_1 - phi))],
                                 [np.cos(tetha_1 -tetha), (-np.sin(tetha_1 -tetha)), 0]]
            conversion_matrix = np.array(conversion_matrix)#.transpose()

            input_matrix = [df_array[i], tetha, phi]
            input_matrix = np.array(input_matrix).transpose()

            buffer_array.append(np.matmul(conversion_matrix, input_matrix))
            #buffer_array.append(np.matmul(conversion_matrix, np.matmul(conversion_matrix, input_matrix)))

        buffer_array = np.array(buffer_array)

        return buffer_array

    def return_true_arrays(self, dataframe, df_array, axis):
        pi = math.pi
        if axis == 'x':
            tetha, phi = ("yaw", "roll")
        elif axis == 'y':
            tetha, phi = ("pitch", "yaw")
        elif axis == 'z':
            tetha, phi = ("roll", "pitch")
        else:
            exit('Input not recognized. Please input, x, y or z for axis')

        pi = math.pi

        buffer_array = []
        for i in range(len(df_array)-1):
            """x = df_array[i] * np.sin(dataframe[tetha][i + 1] - dataframe[tetha][i]) * np.cos(dataframe[phi][i + 1] - dataframe[phi][i])
            y = df_array[i] * np.sin(dataframe[tetha][i + 1] - dataframe[tetha][i]) * np.sin(dataframe[phi][i + 1] - dataframe[phi][i])
            z = df_array[i] * np.cos(dataframe[tetha][i + 1] - dataframe[tetha][i])"""
            x = df_array[i] * np.sin(dataframe[tetha][i]) * np.cos(dataframe[phi][i])
            y = df_array[i] * np.sin(dataframe[tetha][i]) * np.sin(dataframe[phi][i])
            z = df_array[i] * np.cos(dataframe[tetha][i])
            buffer_array.append([x, y, z])

        buffer_array = np.array(buffer_array)
        return buffer_array

    def sum_axes(self, x_buffer, y_buffer, z_buffer):
        x_total = np.random.random(len(x_buffer))
        y_total = np.random.random(len(y_buffer))
        z_total = np.random.random(len(z_buffer))

        total_list = [x_total, y_total, z_total]

        """for i in range(len(x_buffer)):
            x_total[i] = x_buffer[i][0] + y_buffer[i][1] + z_buffer[i][2]
            y_total[i] = x_buffer[i][2] + y_buffer[i][0] + z_buffer[i][1]
            z_total[i] = x_buffer[i][1] + y_buffer[i][2] + z_buffer[i][0]"""


        for i in range(len(x_buffer)):
            for j in range(3):
                total_list[j][i] = x_buffer[i][j] + y_buffer[i][j] + z_buffer[i][j]

        return x_total, y_total, z_total

    def save_sensor_to_dir_as_csv_with_exact_naming(self, sensor_list, save_dir):
        for i in sensor_list:
            file_name = ((os.path.split(i)[1]).split('.')[0])
            saving_dir = f"{save_dir}/QRPfRA_sensorCSV/"
            saving_path = f"{file_name}.csv"
            path = os.path.join(saving_dir, saving_path)
            dataframe = self.read_and_return_pd(file_path=i)

            if os.path.exists(saving_dir):
                dataframe.to_csv(path, index=False)
                print(f"Dataframe saved to {path} as CSV")
            else:
                os.makedirs(saving_dir)
                dataframe.to_csv(path, index=False)
                print(f"Dataframe saved to {path} as CSV")
        return None