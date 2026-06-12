import cv2
import numpy as np
from ultralytics import YOLO
import math

import write
model = YOLO("best.pt") 

IMAGE_PATH = "unnamed (1) (1).jpg" 

TABLE_WIDTH_MM = 600.0   
TABLE_HEIGHT_MM = 340.0  


TOP_LEFT_PIXEL     = [149, 249]
TOP_RIGHT_PIXEL    = [1118, 215]
BOTTOM_RIGHT_PIXEL = [1138, 643]
BOTTOM_LEFT_PIXEL  = [160, 669]

src_pts = np.float32([TOP_LEFT_PIXEL, TOP_RIGHT_PIXEL, BOTTOM_RIGHT_PIXEL, BOTTOM_LEFT_PIXEL])
dst_pts = np.float32([[0, 0], [TABLE_WIDTH_MM, 0], [TABLE_WIDTH_MM, TABLE_HEIGHT_MM], [0, TABLE_HEIGHT_MM]])
homography_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

frame = cv2.imread(IMAGE_PATH)

if frame is not None:
    frame = cv2.resize(frame, (1280, 720))

if frame is None:
    print(f"Error: Could not look up or read the file at '{IMAGE_PATH}'. Check the filename!")
else:
    # Run AI inference directly on the single image
    results = model(frame, verbose=False)

    print(f"--- Processing Results for {IMAGE_PATH} ---")
    
    # Parse the detection outputs
    for r in results:
        if len(r.boxes) == 0:
            print("No eraser detected in this frame.")
            
        for box in r.boxes:
            # Extract bounding box corners
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            confidence = float(box.conf[0])
            
            # Calculate pixel center point
            pixel_x = (x1 + x2) // 2
            pixel_y = (y1 + y2) // 2
            
            # --- HOMOGRAPHY CONVERSION ---
            pixel_point = np.array([[[pixel_x, pixel_y]]], dtype=np.float32)
            real_world_point = cv2.perspectiveTransform(pixel_point, homography_matrix)
# ... (Your homography math) ...
            mm_x = real_world_point[0][0][0]
            mm_y = real_world_point[0][0][1]
            
            # Print data payload to terminal
            print(f"Eraser Found!")
            print(f"  -> Table Position:  X={mm_x:.1f}mm, Y={380.0 - mm_y:.1f}mm")
            
            # --- SEND TO ESP32 ---
            # Set target Z-offset (e.g., 10mm off the table). 
            # Send 'u' to make sure the gripper stays open as it goes to pick it up.
            target_z = 10.0 

            x = 600 - mm_x
            if (x>300):
                x = x-300
                x = x*(-1)

            x = x/10

            y = 340 - mm_y



            ground_dist = math.sqrt(math.pow(x, 2) + math.pow(y + 2.5, 2))
            length = math.sqrt(math.pow(ground_dist, 2) + math.pow(target_z - 5, 2))

            if (length < 30):
                write.writeSerial(mm_x, 340.0 - mm_y, target_z, command='u')
                print("Sent")
            else:
                print("This coordinate is not within range")
                write.writeSerial(mm_x, 340.0 - mm_y, target_z, command='u')
            
            # Draw visual tracking guides (Green box, Red dot)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (pixel_x, pixel_y), 5, (0, 0, 255), -1)
            ui_label = f"X: {mm_x:.1f}mm Y: {mm_y:.1f}mm"
            cv2.putText(frame, ui_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # --- DYNAMIC CALIBRATION CHECK: BLUE BOUNDING BOX ---
    # This takes your hardcoded coordinates and draws them dynamically on the screen
    calibration_pts = np.array([TOP_LEFT_PIXEL, TOP_RIGHT_PIXEL, BOTTOM_RIGHT_PIXEL, BOTTOM_LEFT_PIXEL], np.int32)
    calibration_pts = calibration_pts.reshape((-1, 1, 2))
    cv2.polylines(frame, [calibration_pts], isClosed=True, color=(255, 0, 0), thickness=2)

    # Display the output confirmation image
    cv2.imshow("Static Image AI Localization", frame)
    print("\nPress any key while looking at the image window to close it.")
    cv2.waitKey(0)
    cv2.destroyAllWindows()
