import cv2
import numpy as np
from ultralytics import YOLO

# 1. Initialize your custom trained model weights
model = YOLO("best.pt") 

# 2. Define the path to the image you want to test
IMAGE_PATH = "image (3).webp" 

# 3. Physical dimensions of your active white table grid (in mm)
# Now that the image is rotated upright, X is the 600mm side, Y is the 380mm side
TABLE_WIDTH_MM = 600.0   # The long dimension running horizontally
TABLE_HEIGHT_MM = 380.0  # The short dimension running vertically

# 4. Paste your brand new upright clicked coordinates here
# 4. Corrected Upright Pixel Corners (Mapped to match your screen window visually)
TOP_LEFT_PIXEL     = [140, 44]    # True visual top-left corner
TOP_RIGHT_PIXEL    = [1148, 77]   # True visual top-right corner
BOTTOM_RIGHT_PIXEL = [1130, 506]  # True visual bottom-right corner
BOTTOM_LEFT_PIXEL  = [123, 482]   # True visual bottom-left corner

  

# Compile calibration sets and compute the perspective transformation matrix
src_pts = np.float32([TOP_LEFT_PIXEL, TOP_RIGHT_PIXEL, BOTTOM_RIGHT_PIXEL, BOTTOM_LEFT_PIXEL])
dst_pts = np.float32([[0, 0], [TABLE_WIDTH_MM, 0], [TABLE_WIDTH_MM, TABLE_HEIGHT_MM], [0, TABLE_HEIGHT_MM]])
homography_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)

# Load the static image file into memory
frame = cv2.imread(IMAGE_PATH)

if frame is not None:
    # Rotate the image 90 degrees clockwise to match your calibration layout
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    frame = cv2.resize(frame, (1280, 720))
else:
    print(f"Error: Could not look up or read the file at '{IMAGE_PATH}'. Check the filename!")
    exit()

# Run AI inference directly on the single upright image
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
        
        # Raw measurements from the top-left origin box (0 to 600mm, and 0 to 380mm)
        raw_mm_x = real_world_point[0][0][0]
        raw_mm_y = real_world_point[0][0][1]
        
        # --- CENTIMETER MATRIX TRANSFORMS ---
        # 1. Shift X origin relative to the 300mm centerline, then convert to cm
        cm_x = (raw_mm_x - 300.0) / 10.0  
        # 2. Convert Y directly to cm
        cm_y = raw_mm_y / 10.0

        # Print data payload to terminal
        print(f"Eraser Found!")
        print(f"  -> Pixel Position:  X={pixel_x}, Y={pixel_y}")
        print(f"  -> Table Position:  X={cm_x:.1f}cm, Y={cm_y:.1f}cm")
        print(f"  -> Confidence:      {confidence:.2%}")
        
        # Draw visual tracking guides (Green box, Red dot)
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(frame, (pixel_x, pixel_y), 5, (0, 0, 255), -1)
        ui_label = f"X: {cm_x:.1f}cm Y: {cm_y:.1f}cm"
        cv2.putText(frame, ui_label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

# --- DYNAMIC CALIBRATION CHECK: BLUE BOUNDING BOX ---
calibration_pts = np.array([TOP_LEFT_PIXEL, TOP_RIGHT_PIXEL, BOTTOM_RIGHT_PIXEL, BOTTOM_LEFT_PIXEL], np.int32)
calibration_pts = calibration_pts.reshape((-1, 1, 2))
cv2.polylines(frame, [calibration_pts], isClosed=True, color=(255, 0, 0), thickness=2)

# Display the output confirmation image
cv2.imshow("Static Image AI Localization", frame)
print("\nPress any key while looking at the image window to close it.")
cv2.waitKey(0)
cv2.destroyAllWindows()
