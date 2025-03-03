import numpy as np
import matplotlib.pyplot as plt

# Define the map from the given text
map_text = [
    "..........@.....",
    "@...@.@@........",
    "..............@.",
    ".....@.@........",
    ".@@...........@@",
    "......@..@.@..@@",
    "......@..@@.....",
    ".@...@.........."
]

# Define the map size
height = len(map_text)
width = len(map_text[0])

# Create an empty numpy array for the image (0 for open space, 1 for obstacles)
map_array = np.zeros((height, width))

# Convert map text to numerical array (1 for obstacles, 0 for open spaces)
for i, row in enumerate(map_text):
    for j, cell in enumerate(row):
        if cell == '@':
            map_array[i, j] = 1  # Mark obstacles

# Plot the map
plt.figure(figsize=(width / 2, height / 2))
plt.imshow(map_array, cmap="gray_r", origin="upper")

# Remove axis labels
plt.xticks([])
plt.yticks([])
plt.tight_layout()

# Show the map
# plt.show()
plt.savefig("2d-map.png")