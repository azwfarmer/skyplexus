import torch

# Normalize RGB values
r, g, b = 126 / 255.0, 124 / 255.0, 129 / 255.0

# Pack into a tensor, scale to [0, 255], and convert to uint8
rgb_tensor = torch.tensor([r, g, b], dtype=torch.float32) * 255
rgb_bytes = rgb_tensor.to(torch.uint8)

# Convert to a single hex string
hex_key = rgb_bytes.view(-1).numpy().tobytes().hex()

print("Hex Key:", hex_key)
