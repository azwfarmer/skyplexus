import torch

def rgb_to_xyz( r, g, b):
    # Normalize the RGB values to [0, 1]
    r = r / 255.0
    g = g / 255.0
    b = b / 255.0
    
    # Apply gamma correction (sRGB to linear RGB)
    def gamma_correction(c):
        return torch.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
    
    r = gamma_correction(r)
    g = gamma_correction(g)
    b = gamma_correction(b)
    
    # Convert RGB to XYZ using the standard RGB to XYZ transformation matrix
    M = torch.tensor([[0.4124564, 0.3575761, 0.1804375],
                    [0.2126729, 0.7151522, 0.0721750],
                    [0.0193339, 0.1191920, 0.9503041]], device=r.device)
    
    x = r * M[0, 0] + g * M[0, 1] + b * M[0, 2]
    y = r * M[1, 0] + g * M[1, 1] + b * M[1, 2]
    z = r * M[2, 0] + g * M[2, 1] + b * M[2, 2]
    
    return x, y, z

def xyz_to_lab( x, y, z):
    # Define the reference white point (D65)
    ref_x = 0.95047
    ref_y = 1.00000
    ref_z = 1.08883
    
    # Normalize XYZ by the reference white point
    x /= ref_x
    y /= ref_y
    z /= ref_z
    
    # Apply the transformation to get Lab
    def f(t):
        return torch.where(t > 0.008856, torch.pow(t, 1/3), (t / 903.3) + 16 / 116)
    
    fx = f(x)
    fy = f(y)
    fz = f(z)
    
    l = (116 * fy) - 16
    a = 500 * (fx - fy)
    b = 200 * (fy - fz)
    
    return l, a, b

def rgb_to_lab( r, g, b):
    x, y, z =  rgb_to_xyz(r, g, b)
    return  xyz_to_lab(x, y, z)

def lab_to_xyz( l, a, b):
    # Define the reference white point (D65)
    ref_x = 0.95047
    ref_y = 1.00000
    ref_z = 1.08883
    
    # Inverse of the f function used in Lab to XYZ
    def f_inv(t):
        return torch.where(t > 0.206893034, t ** 3, (t - 16 / 116) / 0.008856)
    
    fy = (l + 16) / 116
    fx = a / 500 + fy
    fz = fy - b / 200
    
    # Convert to XYZ
    x = f_inv(fx) * ref_x
    y = f_inv(fy) * ref_y
    z = f_inv(fz) * ref_z
    
    return x, y, z

def xyz_to_rgb( x, y, z):
    # Convert XYZ to RGB using the inverse of the RGB to XYZ matrix
    M_inv = torch.tensor([[3.2404542, -1.5371385, -0.4985314],
                        [-0.9692660, 1.8760108, 0.0415560],
                        [0.0556434, -0.2040259, 1.0572252]], device=x.device)
    
    r = x * M_inv[0, 0] + y * M_inv[0, 1] + z * M_inv[0, 2]
    g = x * M_inv[1, 0] + y * M_inv[1, 1] + z * M_inv[1, 2]
    b = x * M_inv[2, 0] + y * M_inv[2, 1] + z * M_inv[2, 2]
    
    # Apply gamma correction (linear RGB to sRGB)
    def gamma_correction_inv( c):
        return torch.where(c <= 0.0031308, c * 12.92, 1.055 * (c ** (1 / 2.4)) - 0.055)
    
    r =  gamma_correction_inv(r)
    g =  gamma_correction_inv(g)
    b =  gamma_correction_inv(b)
    
    # Clip to [0, 1] range and then scale back to [0, 255]
    r = torch.clamp(r, 0, 1) * 255
    g = torch.clamp(g, 0, 1) * 255
    b = torch.clamp(b, 0, 1) * 255
    
    return r, g, b

def lab_to_rgb(l, a, b):
    x, y, z =  lab_to_xyz(l, a, b)        
    return  xyz_to_rgb(x, y, z)

r = torch.tensor([99.0,100.0])
g = torch.tensor([100.0,99.0])
b = torch.tensor([101.0, 98.0])

l, a, b = rgb_to_lab(r, g, b)
print(l, a, b)
r2, g2, b2 = lab_to_rgb(l, a, b)
print(r2, g2, b2)
