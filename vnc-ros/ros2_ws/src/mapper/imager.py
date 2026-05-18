import numpy as np
from PIL import Image

# getting the map data
path = r'ros2_ws\src\map_data_pa4.npy'
grid = np.load(path)

# We create an empty RGB image with the same dimensions as the map
h, w = grid.shape
img_rgb = np.zeros((h, w, 3), dtype=np.uint8)

display_grid = grid.astype(np.float32)

# grayscale for known values
gray = (255 - 255*np.clip(display_grid, 0, 100)/100).astype(np.uint8)

# convert grayscale to RGB
img_rgb = np.stack([gray]*3, axis=-1)

# unknown cells become red
img_rgb[grid == -1] = [100, 0, 0]

# getting the path points data
path = r'ros2_ws\src\waypoint_path_data_pa4.npy'
points = np.load(path)

# set path points to red
for x, y in points:
    img_rgb[y, x] = [0, 200, 0]

img_rgb = np.flipud(img_rgb)

# show image
img = Image.fromarray(img_rgb)
img.save('path.png')
img.show()
