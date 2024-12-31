import torch

r, g, b = torch.tensor(126, dtype=torch.uint8), torch.tensor(124, dtype=torch.uint8), torch.tensor(129, dtype=torch.uint8)
grid_rgb_flat = (r.to(torch.int32) << 16) | (g.to(torch.int32) << 8) | b.to(torch.int32).item()
print(grid_rgb_flat)
print(hex(grid_rgb_flat))