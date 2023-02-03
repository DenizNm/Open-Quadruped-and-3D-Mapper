import matplotlib.pyplot as plt
import numpy as np
import firebase_admin
from firebase_admin import credentials
from firebase_admin import storage
from firebase_admin import db
from pillow_heif import register_heif_opener
import cameraFunctions as camera
from captureStateFirebase import captureState
desktop_path = ".../Desktop"
dataBaseURL = "database-url"
storageBucket = "storage-bucket"
cred = credentials.Certificate('certificate.json')
firebase_admin.initialize_app(cred, {
    'databaseURL': dataBaseURL,
    'storageURL': storageBucket}
)
register_heif_opener()
ref = db.reference('capture')
bucket = storage.bucket("depthstreamer.appspot.com")


"""--------------------SETTING CAPTURE STATE AND INITIALIZING CLASSES--------------------"""
captureState(True, ref)
camera_functions = camera.camera_funcitons(bucket=bucket)
matrix_operations = camera.matrix_operations(np=np)
point_cloud_functions = camera.point_cloud_functions(np=np, camera_func=camera_functions, matrix_op=matrix_operations)
sensor_preprocessing = camera.sensor_preprocessing(bucket=bucket)

"""--------------------CLEAR DATABASE FOR GIVEN DAY--------------------"""
#camera_functions.clearDatabase_for_given_date("04-08-22")


"""--------------------DOWNLAOD DATASET IF NECESSARY--------------------"""

saving_directory = ".../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications"
############################image_dataset, depth_dataset, sensor_dataset = camera_functions.download_jpg_or_depth_to_dir("26-08-22", saving_directory, download_depth=True)


"""--------------------READING IMAGE-DEPTH DIRECTORY  AND LISTING ELEMENTS--------------------"""
image_dataset = ".../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_image_dataset"
depth_dataset = ".../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_depth_dataset"
sensor_dataset = ".../Desktop/QRPfRA-Quadruple_Research_Platform_for_Robotic_Applications/QRPfRA_sensor_dataset"
img_list, depth_list, sensor_list = camera_functions.list_of_image_and_depth_files(image_dataset,depth_dataset,sensor_dataset)


hh_mm_ss = "19-32-55"



######example_depth_map = camera_functions.return_example_depth_data_nparray_from_firebase("06-08-22", "09-51-03")
print("Getting files from firebase or on-device files")


"""--------------------READING FROM EXISTING LIST--------------------"""
##################example_depth_map = camera_functions.return_example_depth_data_nparray_from_existing_file(depth_list,randomized=False, hh_mm_ss=hh_mm_ss)
print("Test1")
##################print("Example depth map shape",example_depth_map.shape)


"""--------------------APPLY UNDISTORTION IF NECESSARY--------------------"""
#example_depth_map = camera_functions.undistored_image(example_depth_map)


"""--------------------PREPROCESS DEPTH_MAP TO POINT_CLOUD--------------------"""
print("Converting depth_map to point cloud.....\n")
#####################point_ready_depth_map = camera_functions.return_point_cloud_ready_nparray(example_depth_map, depth_list_element_name=hh_mm_ss)
print("Converted\n")

"""--------------------CONVERTING POINT_CLOUD TO CAMERA POV POINT_CLOUD--------------------"""
"""Applying cartesian conversion to point_cloud array."""
################converted_point_cloud = matrix_operations.spherical_to_cartesian_conversion(point_ready_depth_map, depth_filter=True, depth_max_cm=300, depth_min_cm=20)
####converted_point_cloud = matrix_operations.point_conversion(point_ready_depth_map)


"""--------------------NORMALIZING POINT_CLOUD TO CAMERA POV POINT_CLOUD WITH --------------------"""
#positive_point_cloud = point_cloud_functions.normalized_cloud_array(converted_point_cloud)

#transformed_point_cloud = point_cloud_functions.transform_normalized_cloud_array(converted_point_cloud)
"""--------------------ASSIGNING CONVERTED_POINT_CLOUD TO POINT_CLOUD_READY--------------------"""
################point_ready_depth_map = converted_point_cloud

"""--------------------SAVE TO CSV--------------------"""
###point_cloud_functions.save_point_cloud_to_csv(converted_point_cloud, desktop_path, name=None, floating_point_precision=0)

"""--------------------PLOT AND SHOW 3D GEOMETRY--------------------"""
#################camera_functions.plot_and_show_3d_geometry(example_depth_map,point_ready_depth_map, show3D=True)

"""--------------------READ SENSOR DATA AS PANDAS DATAFRAME--------------------"""
######sensor_path = "/Users/deniz/Desktop/QRPfRA_sensor_dataset/16-51-07-10.json"
######pandas_df = sensor_preprocessing.read_and_return_pd(from_firebase=True,date_of_interest="05-08-22",hh_mm_ss="16-50-37")#, date_of_interest="25-07-22",hh_mm_ss="14-50-54")

#x, y, z = sensor_preprocessing.get_accel_displacement(dataframe=pandas_df)
#print("Total Change:", math.sqrt((x**2 + y**2 + z**2)))
#####sensor_preprocessing.write_pd_to_csv(dataframe=pandas_df, saving_path=desktop_path)
#print(f"Net Change in (cm) X:{x}, Y:{y}, Z:{z}")

"""--------------------SAVE SENSOR INFO TO CSV--------------------"""
############################sensor_preprocessing.save_sensor_to_dir_as_csv_with_exact_naming(sensor_list, saving_directory)
######sensor_preprocessing.write_pd_to_csv(pandas_df, desktop_path)

"""--------------------SAVE COLOR POINT ARRAY TO CSV--------------------"""
#point_cloud_functions.color_array_for_point_cloud_to_csv()
point_cloud_functions.save_point_cloud_to_dir_as_csv_with_exact_naming(depth_list, save_dir=saving_directory, depth_filter=True, depth_max_cm=250)#, delete_bright_spots=True)
