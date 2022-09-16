# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
from robot_movements import *

def main_pipeline(mode = "read", clipping_dist = 1, file_name = None):
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)
    
    if mode == "record":
        config.enable_record_to_file(file_name)
    elif mode == "read":
        config.enable_device_from_file(file_name)


    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'L500':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)
    profile1 = profile.get_stream(rs.stream.color)
    intr = profile1.as_video_stream_profile().get_intrinsics()

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)

    # We will be removing the background of objects more than
    #  clipping_distance_in_meters meters away
    clipping_distance_in_meters = clipping_dist #1 meter
    clipping_distance = clipping_distance_in_meters / depth_scale

    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)

    # Streaming loop
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())
            # print(depth_image.shape)
            color_image = np.asanyarray(color_frame.get_data())
            # print(depth_image.shape)

            # Remove background - Set pixels further than clipping_distance to grey
            grey_color = 153
            depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) #depth image is 1 channel, color is 3 channels
            bg_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), grey_color, color_image)
            # Convert to HSV
            bg_removed = cv2.cvtColor(bg_removed, cv2.COLOR_BGR2HSV)
            # Only keep purple color
            bg_removed = cv2.inRange(bg_removed, (126,76,47), (152, 223, 255))
            
            # BGR_images = cv2.cvtColor(bg_removed, cv2.COLOR_HSV2BGR)
            # dilation_size = 7
            
            # Filter image using dilatation
            bg_removed = dilatation(bg_removed)
            bg_removed = erosion(bg_removed)

            # Render images:
            #   depth align to color on left
            #   depth on right
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)


            # images = np.hstack((bg_removed, depth_colormap))
            images = bg_removed
            # _,_,images = cv2.split(images)
            
            # images = cv2.Canny(images, 80, 255)
            # imgray = cv2.cvtColor(BGR_images, cv2.COLOR_BGR2GRAY)
            
            contours, hierarchy = cv2.findContours(images, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.imshow('BG Removed', images)
            # cv2.namedWindow('align', cv2.WINDOW_NORMAL)
            cv2.drawContours(color_image, contours, -1, (0,255,0), 3)

            # calculate centroid here
            if contours:
                max_area = float("-inf")
                main_body = contours[0]
                for cnt in contours:
                    if cv2.contourArea(cnt) > max_area:
                        max_area = cv2.contourArea(cnt)
                        main_body = cnt
                M = cv2.moments(main_body)
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(color_image, (cx, cy), radius = 5, color = (0,0,255), thickness = -1)
                x_coord, y_coord, depth = rs.rs2_deproject_pixel_to_point(intr, [cx,cy], depth_image[cy][cx] * depth_scale)
                # print(str(x_coord) + " " + str(y_coord) + " " + str(depth))
            
                waist_motion(x_coord, depth)

            cv2.imshow('Contours', color_image)

            key = cv2.waitKey(1)
            # Press esc or 'q' to close the image window
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
    finally:
        pipeline.stop()

def dilatation(images):
    dilatation_size = 12
    dilation_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(dilation_shape, (2 * dilatation_size + 1, 2 * dilatation_size + 1),
                                       (dilatation_size, dilatation_size))
    dilatation_dst = cv2.dilate(images, element)
    return dilatation_dst
    # cv.imshow(title_dilation_window, dilatation_dst)

def erosion(images):
    erosion_size = 1
    erosion_shape = cv2.MORPH_ELLIPSE
    element = cv2.getStructuringElement(erosion_shape, (2 * erosion_size + 1, 2 * erosion_size + 1),
                                       (erosion_size, erosion_size))
    erosion_dst = cv2.erode(images, element)
    return erosion_dst