## *********************************************************
## 
## File autogenerated for the mv_camera package 
## by the dynamic_reconfigure package.
## Please do not edit.
## 
## ********************************************************/

from dynamic_reconfigure.encoding import extract_params

inf = float('inf')

config_description = {'upper': 'DEFAULT', 'lower': 'groups', 'srcline': 233, 'name': 'Default', 'parent': 0, 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'cstate': 'true', 'parentname': 'Default', 'class': 'DEFAULT', 'field': 'default', 'state': True, 'parentclass': '', 'groups': [], 'parameters': [{'srcline': 259, 'description': 'Serial number of camera, suffix CAMERA_MODEL_ + serial number (use first camera if null). e.g. BLUECOUGAR_GX002408', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'guid', 'edit_method': '', 'default': '', 'level': 3, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'ROS tf frame of reference, resolved with tf_prefix unless absolute.', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'frame_id', 'edit_method': '', 'default': 'camera', 'level': 3, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Pixel clock of image sensor [MHz].', 'max': 57.6, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'pixel_clock', 'edit_method': "{'enum_description': 'Pixel clocks.', 'enum': [{'srcline': 61, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 6.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_6M'}, {'srcline': 62, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 8.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_8M'}, {'srcline': 63, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 10.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_10M'}, {'srcline': 64, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 13.5, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_13M5'}, {'srcline': 65, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 20.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_20M'}, {'srcline': 66, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 24.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_24M'}, {'srcline': 67, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 27.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_27M'}, {'srcline': 68, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 32.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_32M'}, {'srcline': 69, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 37.6, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_37M6'}, {'srcline': 70, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 40.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_40M'}, {'srcline': 71, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 66.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_66M'}, {'srcline': 72, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 50.0, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_50M'}, {'srcline': 73, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const double', 'value': 57.6, 'ctype': 'double', 'type': 'double', 'name': 'pixel_clock_57M6'}]}", 'default': 40.0, 'level': 0, 'min': 6.0, 'type': 'double'}, {'srcline': 259, 'description': 'Camera speed (frames per second).', 'max': 240.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'frame_rate', 'edit_method': '', 'default': 15.0, 'level': 0, 'min': 1.0, 'type': 'double'}, {'srcline': 259, 'description': 'Camera calibration URL for this video_mode (uncalibrated if null).', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'camera_info_url', 'edit_method': '', 'default': '', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Number of pixels combined for horizontal binning, use device hints if zero.', 'max': 4, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'binning_x', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Number of pixels combined for vertical binning, use device hints if zero.', 'max': 4, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'binning_y', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Width of Region of Interest in unbinned pixels, full width if zero.', 'max': 5000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'roi_width', 'edit_method': '', 'default': 5000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Height of Region of Interest in unbinned pixels, full height if zero.', 'max': 5000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'roi_height', 'edit_method': '', 'default': 5000, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Horizontal offset for left side of ROI in unbinned pixels.', 'max': 5000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'x_offset', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Vertical offset for top of ROI in unbinned pixels.', 'max': 5000, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'y_offset', 'edit_method': '', 'default': 0, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Color coding', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'color_coding', 'edit_method': "{'enum_description': 'color coding methods', 'enum': [{'srcline': 112, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'mono8', 'ctype': 'std::string', 'type': 'str', 'name': 'mono8'}, {'srcline': 113, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'mono16', 'ctype': 'std::string', 'type': 'str', 'name': 'mono16'}, {'srcline': 114, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'raw8', 'ctype': 'std::string', 'type': 'str', 'name': 'raw8'}, {'srcline': 115, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'bgr8', 'ctype': 'std::string', 'type': 'str', 'name': 'bgr8'}, {'srcline': 116, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'bgra8', 'ctype': 'std::string', 'type': 'str', 'name': 'bgra8'}, {'srcline': 117, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'bgr16', 'ctype': 'std::string', 'type': 'str', 'name': 'bgr16'}, {'srcline': 118, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'auto', 'ctype': 'std::string', 'type': 'str', 'name': 'auto'}]}", 'default': 'auto', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Bayer decoding method (default: ROS image_proc).', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'bayer_method', 'edit_method': "{'enum_description': 'Bayer method', 'enum': [{'srcline': 127, 'description': 'Decode via ROS image_proc', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': '', 'ctype': 'std::string', 'type': 'str', 'name': 'image_proc'}, {'srcline': 128, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'DownSample', 'ctype': 'std::string', 'type': 'str', 'name': 'DownSample'}, {'srcline': 129, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Simple', 'ctype': 'std::string', 'type': 'str', 'name': 'Simple'}, {'srcline': 130, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Bilinear', 'ctype': 'std::string', 'type': 'str', 'name': 'Bilinear'}, {'srcline': 131, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'HQ', 'ctype': 'std::string', 'type': 'str', 'name': 'HQ'}, {'srcline': 132, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'VNG', 'ctype': 'std::string', 'type': 'str', 'name': 'VNG'}, {'srcline': 133, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'AHD', 'ctype': 'std::string', 'type': 'str', 'name': 'AHD'}]}", 'default': '', 'level': 3, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Auto exposure value .', 'max': 255, 'cconsttype': 'const int', 'ctype': 'int', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'exposure', 'edit_method': '', 'default': 100, 'level': 0, 'min': 0, 'type': 'int'}, {'srcline': 259, 'description': 'Shutter control state.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'shutter_auto', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Min shutter time [s] in auto mode.', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'shutter_auto_min', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Max shutter time [s] in auto mode', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'shutter_auto_max', 'edit_method': '', 'default': 0.1, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Shutter time [s].', 'max': 0.1, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'shutter', 'edit_method': '', 'default': 0.005, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Gain control state.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'gain_auto', 'edit_method': '', 'default': True, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Min relative circuit gain [dB] in auto mode.', 'max': 12.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'gain_auto_min', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Max relative circuit gain [dB] in auto mode.', 'max': 12.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'gain_auto_max', 'edit_method': '', 'default': 12.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Relative circuit gain [dB] ', 'max': 12.0, 'cconsttype': 'const double', 'ctype': 'double', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'gain', 'edit_method': '', 'default': 0.0, 'level': 0, 'min': 0.0, 'type': 'double'}, {'srcline': 259, 'description': 'Control speed for automatic features', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'auto_control_speed', 'edit_method': "{'enum_description': 'Control speed for automatic features', 'enum': [{'srcline': 149, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Slow', 'ctype': 'std::string', 'type': 'str', 'name': 'acs_slow'}, {'srcline': 150, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Medium', 'ctype': 'std::string', 'type': 'str', 'name': 'acs_medium'}, {'srcline': 151, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fast', 'ctype': 'std::string', 'type': 'str', 'name': 'acs_fast'}]}", 'default': 'Medium', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Queries settings that are auto controlled.', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'auto_query_values', 'edit_method': '', 'default': False, 'level': 0, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'HDR mode', 'max': '', 'cconsttype': 'const char * const', 'ctype': 'std::string', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'hdr_mode', 'edit_method': "{'enum_description': 'Enable HDR / set HDR mode', 'enum': [{'srcline': 195, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'off', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_off'}, {'srcline': 196, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fixed0', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_fixed0'}, {'srcline': 197, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fixed1', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_fixed1'}, {'srcline': 198, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fixed2', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_fixed2'}, {'srcline': 199, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fixed3', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_fixed3'}, {'srcline': 200, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fixed4', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_fixed4'}, {'srcline': 201, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fixed5', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_fixed5'}, {'srcline': 202, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'Fixed6', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_fixed6'}, {'srcline': 203, 'description': '', 'srcfile': '../cfg/mv_camera.cfg', 'cconsttype': 'const char * const', 'value': 'User', 'ctype': 'std::string', 'type': 'str', 'name': 'hdr_user'}]}", 'default': 'off', 'level': 0, 'min': '', 'type': 'str'}, {'srcline': 259, 'description': 'Timestamp Image and CameraInfo using ros::Time::now()', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'use_ros_time', 'edit_method': '', 'default': True, 'level': 3, 'min': False, 'type': 'bool'}, {'srcline': 259, 'description': 'Embed extra info into the image data', 'max': True, 'cconsttype': 'const bool', 'ctype': 'bool', 'srcfile': '/opt/ros/indigo/lib/python2.7/dist-packages/dynamic_reconfigure/parameter_generator.py', 'name': 'embed_image_info', 'edit_method': '', 'default': True, 'level': 3, 'min': False, 'type': 'bool'}], 'type': '', 'id': 0}

min = {}
max = {}
defaults = {}
level = {}
type = {}
all_level = 0

#def extract_params(config):
#    params = []
#    params.extend(config['parameters'])    
#    for group in config['groups']:
#        params.extend(extract_params(group))
#    return params

for param in extract_params(config_description):
    min[param['name']] = param['min']
    max[param['name']] = param['max']
    defaults[param['name']] = param['default']
    level[param['name']] = param['level']
    type[param['name']] = param['type']
    all_level = all_level | param['level']

MVCamera_pixel_clock_6M = 6.0
MVCamera_pixel_clock_8M = 8.0
MVCamera_pixel_clock_10M = 10.0
MVCamera_pixel_clock_13M5 = 13.5
MVCamera_pixel_clock_20M = 20.0
MVCamera_pixel_clock_24M = 24.0
MVCamera_pixel_clock_27M = 27.0
MVCamera_pixel_clock_32M = 32.0
MVCamera_pixel_clock_37M6 = 37.6
MVCamera_pixel_clock_40M = 40.0
MVCamera_pixel_clock_66M = 66.0
MVCamera_pixel_clock_50M = 50.0
MVCamera_pixel_clock_57M6 = 57.6
MVCamera_mono8 = 'mono8'
MVCamera_mono16 = 'mono16'
MVCamera_raw8 = 'raw8'
MVCamera_bgr8 = 'bgr8'
MVCamera_bgra8 = 'bgra8'
MVCamera_bgr16 = 'bgr16'
MVCamera_auto = 'auto'
MVCamera_image_proc = ''
MVCamera_DownSample = 'DownSample'
MVCamera_Simple = 'Simple'
MVCamera_Bilinear = 'Bilinear'
MVCamera_HQ = 'HQ'
MVCamera_VNG = 'VNG'
MVCamera_AHD = 'AHD'
MVCamera_Off = 0
MVCamera_Query = 1
MVCamera_Auto = 2
MVCamera_Manual = 3
MVCamera_OnePush = 4
MVCamera_None = 5
MVCamera_acs_slow = 'Slow'
MVCamera_acs_medium = 'Medium'
MVCamera_acs_fast = 'Fast'
MVCamera_hdr_off = 'off'
MVCamera_hdr_fixed0 = 'Fixed0'
MVCamera_hdr_fixed1 = 'Fixed1'
MVCamera_hdr_fixed2 = 'Fixed2'
MVCamera_hdr_fixed3 = 'Fixed3'
MVCamera_hdr_fixed4 = 'Fixed4'
MVCamera_hdr_fixed5 = 'Fixed5'
MVCamera_hdr_fixed6 = 'Fixed6'
MVCamera_hdr_user = 'User'
