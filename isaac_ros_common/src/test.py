import numpy as np
import colorsys

def hsv_to_rgb(h, s, v):
    h = h.astype(float)
    s = s.astype(float)
    v = v.astype(float)

    r = np.zeros_like(h)
    g = np.zeros_like(h)
    b = np.zeros_like(h)

    mask = (s == 0)
    r[mask] = g[mask] = b[mask] = v[mask] * 255

    not_mask = ~mask
    h_prime = (h[not_mask] * 6).astype(int)
    f = h[not_mask] * 6 - h_prime
    p = v[not_mask] * (1 - s[not_mask])
    q = v[not_mask] * (1 - f * s[not_mask])
    t = v[not_mask] * (1 - (1 - f) * s[not_mask])

    r[not_mask] = np.select(
        [h_prime == 0, h_prime == 1, h_prime == 2, h_prime == 3, h_prime == 4, h_prime == 5],
        [v[not_mask], q, p, p, t, v[not_mask]],
        default=0
    )
    g[not_mask] = np.select(
        [h_prime == 0, h_prime == 1, h_prime == 2, h_prime == 3, h_prime == 4, h_prime == 5],
        [t, v[not_mask], v[not_mask], q, p, p],
        default=0
    )
    b[not_mask] = np.select(
        [h_prime == 0, h_prime == 1, h_prime == 2, h_prime == 3, h_prime == 4, h_prime == 5],
        [p, p, t, v[not_mask], v[not_mask], q],
        default=0
    )

    rgb = np.stack((r, g, b), axis=-1) * 255
    return rgb.astype(np.uint8)

print(hsv_to_rgb(np.array([0.1]),np.array([0.2]),np.array([0.3])))
print(tuple(x * 255 for x in colorsys.hsv_to_rgb(0.1,0.2,0.3)))
