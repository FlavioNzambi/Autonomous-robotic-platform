"""
        Program:    Rover controller with follow me behavior  
         Author:    Flaviano Nzambi
    Description:    Using Jetson Inference and Realsense D435i detects a human checks if he is wearing an orange vest 
                    and then follows the person via controlling the rover has obstacle avoidance capabilities.
           Date:    25th February 2024
"""
import sys
import traceback
import cv2
import h_rovercontrol 
import jetson_utils
import pyrealsense2 as rs 
import h_pid_control
import diskcache as dc
from jetson_inference import detectNet
from pathlib import Path

def main():
    ## VARIABLES
    tmp = Path("/tmp/stream")
    target_x = 640
    roll_offset = 150
    throttle_value = 350

    # THRESHOLDS
    lower_range = (5, 165,130)
    center = 80
    orange_threshold = 5
    safety_distance_threshold = 0.5
    roll_distance_threshold = 1.2
    throttle_distance_threshold = 2.2
    
    pid = h_pid_control.PID_Controller(kd=1,ki=0.5,kp=0.5)

    rover = h_rovercontrol.RoverController()
    rover.connect(address="/dev/ttyACM0",baud=115200)
    net = detectNet("peoplenet", threshold=0.5)
    
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    profile = pipeline.start(config)
    camera = jetson_utils.videoSource("/dev/video4")  

    ## MAIN LOOP

    try:
        rover.arm()
        rover.change_mode("MANUAL")
        rover.stop()

        while True:
            control_values = read_values_from_file()
            if control_values is not None:
                throttle_value = int(control_values.get('throttle'))
                orange_threshold = control_values.get('orangeThreshold')
                roll_value = int(control_values.get('roll'))
                kp = control_values.get('kp')
                ki = control_values.get('ki')
                kd = control_values.get('kd')
            
            # Get Image and frames
            image_cuda = camera.Capture(format='rgb8')
            if image_cuda is None:
                print("Status: no input image recieved")
                continue
            image_array = jetson_utils.cudaToNumpy(image_cuda)
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()

            # Obstacle detection
            obstacle_detected , image_array  = obstacle_detection(depth_frame,image_array,safety_distance_threshold)


            detections = net.Detect(image_cuda,overlay="none")

            if obstacle_detected is True:
                print("Status: Obstacle detected")
                rover.stop()

            elif len(detections) == 0:
                print("Status: No Detections")
                rover.stop()

            else:
                for detection in detections:

                    if detection.ClassID != 0:
                        break

                    detection_distance = depth_frame.get_distance(int(detection.Center[0]),int(detection.Center[1])) 
                    left, top, right, bottom ,current_x = int(detection.Left), int(detection.Top), int(detection.Right), int(detection.Bottom) , int(detection.Center[0])

                    # Orange Detection
                    roi = image_array[top:bottom, left:right]
                    hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)
                    mask = cv2.inRange(hsv, lower_range, (15, 255, 255))
                    total_pixels = roi.shape[0] * roi.shape[1]
                    orange_pixels = cv2.countNonZero(mask)
                    orange_percentage = (orange_pixels / total_pixels) * 100
                    image_array[top:bottom, left:right] = cv2.addWeighted(roi, 1, cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), 0.5, 0)

                    #if 2 > 1: 
                    if orange_percentage > orange_threshold:
                        cv2.rectangle(image_array,(left,top),(right,bottom),(255,0,0),2)
                        cv2.circle(image_array, (int(detection.Center[0]),int(detection.Center[1])) ,4,(255,0,0),4)
                        roll , throttle = 0 , 0

                        if detection_distance > roll_distance_threshold:
                            if (target_x-current_x) > -center:
                                roll = roll_value
                            if (target_x-current_x) < center:
                                roll = -roll_value
                            #roll = pid.calculate_roll(target_x=target_x ,current_x=current_x,time=time.monotonic(),kp=kp,ki=ki,kd=kd,new=True)
                            if detection_distance > throttle_distance_threshold:
                                throttle = throttle_value
                                #rover.rc_send(3,1370)
                                    
                            print("Status: sending roll value :",roll,"throttle:",throttle)
                            rover.forward_turn(forward_value=throttle,turn_value=roll)

                        else:
                            rover.stop()
                    else:   
                        print("Status:Not Enough Orange")   
                        cv2.rectangle(image_array,(left,top),(right,bottom),(0,255,0),2)
                        cv2.circle(image_array, (int(detection.Center[0]),int(detection.Center[1])) ,4,(0,255,0),4)
            
            with dc.Cache(tmp) as cache:
                cache.push(image_array, expire=0.01)

    except KeyboardInterrupt:
        pipeline.stop()
        rover.close()
        print ("Status: Program Shutdown...clearing comms / exiting")
    except Exception:
        pipeline.stop()
        rover.close()
        traceback.print_exc(file=sys.stdout)
    sys.exit(0)
    
def read_values_from_file(filename='config.txt'):
    try:
        with open(filename, 'r') as file:
            lines = file.readlines()
            values = {}
            for line in lines:
                key, value = line.strip().split('=')
                values[key] = float(value)
            return values
    except Exception as e:
        print(f"Error reading values from file: {e}")
        return {'kp': 1, 'ki': 0.5, 'kd': 0.5, 'orangeThreshold': 30, 'roll': 330, 'throttle': 250}
    
def obstacle_detection(depth_frame,image_array,safety_distance_threshold):
    
    grid_rows , grid_columns = 2 , 2
    grid_spacing_x = 1280 // (grid_columns + 1)
    grid_spacing_y = 720 // (grid_rows + 1)
    obstacle_detected = False
    for row in range(2, grid_rows):
        for col in range(2, grid_columns):
            x = int(col * grid_spacing_x)
            y = int(row * grid_spacing_y)

            cv2.circle(image_array, (x, y), 5, (0, 0, 255), -1)
            point_distance = depth_frame.get_distance(x, y)
            if point_distance < safety_distance_threshold:
                obstacle_detected = True
                break  
    
    return obstacle_detected , image_array

if __name__ == "__main__":
    main()
