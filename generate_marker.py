import cv2
import os
from cv2 import aruco

# --- Configuration ---
# Choose a dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)

# Marker ID (0 to 49 for this dictionary)
marker_id = 23

# Size of the output image in pixels
marker_size_px = 300

# Folder where markers will be saved (relative to project root)
output_folder = "ArucoMarkers"
output_filename = f"marker_{marker_id}.png"
output_path = os.path.join(output_folder, output_filename)
# --------------------

# Create the folder if it doesn't exist
os.makedirs(output_folder, exist_ok=True)

# Generate the marker
print(f"Generating marker ID {marker_id}...")
marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size_px)

# Save the image
cv2.imwrite(output_path, marker_image)
print(f"Marker saved as '{output_path}'")

# Optional: Display the marker
cv2.imshow("Generated ArUco Marker", marker_image)
cv2.waitKey(0)
cv2.destroyAllWindows()