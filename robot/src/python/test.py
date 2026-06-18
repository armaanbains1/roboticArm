import cv2

IMAGE_PATH = "image (3).webp"
img = cv2.imread(IMAGE_PATH)

if img is None:
    print(f"Error: Could not read '{IMAGE_PATH}'")
    exit()

# ROTATE THE IMAGE UP_RIGHT SO IT MATCHES REAL LIFE
img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
img = cv2.resize(img, (1280, 720))

corners = []

def click_event(event, x, y, flags, params):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked Pixel Coordinates: [{x}, {y}]")
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.imshow("Click Corners", img)
        corners.append([x, y])
        
        if len(corners) == 4:
            print("\n--- COPY AND PASTE THESE ---")
            print(f"TOP_LEFT_PIXEL     = {corners[3]}")
            print(f"TOP_RIGHT_PIXEL    = {corners[0]}")
            print(f"BOTTOM_RIGHT_PIXEL = {corners[1]}")
            print(f"BOTTOM_LEFT_PIXEL  = {corners[2]}")
            print("----------------------------")

print("Click the 4 corners clockwise starting from the TOP-LEFT of the whiteboard:")
cv2.imshow("Click Corners", img)
cv2.setMouseCallback("Click Corners", click_event)
cv2.waitKey(0)
cv2.destroyAllWindows()
