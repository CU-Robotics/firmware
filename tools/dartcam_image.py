from PIL import Image
import numpy as np

# Read the hex data from 'image.hex'
with open('image.hex', 'r') as file:
    hex_data = file.read()

# Remove any whitespace or newlines
hex_data = ''.join(hex_data.split())

# Convert hex string to bytes
byte_data = bytes.fromhex(hex_data)

# Set image dimensions
width, height = 320, 240

# Ensure the byte data matches the expected size
expected_size = width * height * 2  # RGB565 is 2 bytes per pixel
if len(byte_data) != expected_size:
    raise ValueError(f"Byte data size ({len(byte_data)}) does not match expected size ({expected_size}) for dimensions {width}x{height}")

# Convert RGB565 to RGB888
rgb_array = np.zeros((height, width, 3), dtype=np.uint8)

for i in range(height * width):
    # Extract two bytes for RGB565
    rgb565 = byte_data[2 * i] << 8 | byte_data[2 * i + 1]

    # Convert RGB565 to individual RGB components
    r = (rgb565 >> 11) & 0x1F  # 5 bits for red
    g = (rgb565 >> 5) & 0x3F   # 6 bits for green
    b = rgb565 & 0x1F          # 5 bits for blue

    # Scale to 8-bit values
    r = (r << 3) | (r >> 2)
    g = (g << 2) | (g >> 4)
    b = (b << 3) | (b >> 2)

    # Store in the RGB array
    y = i // width
    x = i % width
    rgb_array[y, x] = [r, g, b]

# Create an image from the RGB array
image = Image.fromarray(rgb_array, 'RGB')

# Save the image
image.save('output.png')
print("Image saved as 'output.png'")

# Display the image
image.show()