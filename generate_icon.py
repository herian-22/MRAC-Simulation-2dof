"""
generate_icon.py — Generate a simple application icon for the MRAC Simulink app.

Creates an ICO file with a parabolic antenna dish design.
"""

import struct
import zlib
import os


def create_png(width, height, pixels):
    """Create a minimal PNG file from RGBA pixel data."""
    def write_chunk(chunk_type, data):
        chunk = chunk_type + data
        return struct.pack('>I', len(data)) + chunk + struct.pack('>I', zlib.crc32(chunk) & 0xFFFFFFFF)

    # PNG signature
    sig = b'\x89PNG\r\n\x1a\n'

    # IHDR
    ihdr_data = struct.pack('>IIBBBBB', width, height, 8, 6, 0, 0, 0)  # 8-bit RGBA
    ihdr = write_chunk(b'IHDR', ihdr_data)

    # IDAT
    raw_data = b''
    for y in range(height):
        raw_data += b'\x00'  # filter byte
        for x in range(width):
            idx = (y * width + x) * 4
            raw_data += bytes(pixels[idx:idx+4])
    compressed = zlib.compress(raw_data)
    idat = write_chunk(b'IDAT', compressed)

    # IEND
    iend = write_chunk(b'IEND', b'')

    return sig + ihdr + idat + iend


def draw_icon(size=256):
    """Draw a parabolic antenna icon with MRAC text."""
    pixels = [0] * (size * size * 4)

    cx, cy = size // 2, size // 2
    r = size // 2 - 4

    def set_pixel(x, y, r, g, b, a=255):
        if 0 <= x < size and 0 <= y < size:
            idx = (y * size + x) * 4
            # Alpha blend
            old_a = pixels[idx + 3]
            if old_a == 0:
                pixels[idx] = r
                pixels[idx + 1] = g
                pixels[idx + 2] = b
                pixels[idx + 3] = a
            else:
                pixels[idx] = min(255, (pixels[idx] * (255 - a) + r * a) // 255)
                pixels[idx + 1] = min(255, (pixels[idx + 1] * (255 - a) + g * a) // 255)
                pixels[idx + 2] = min(255, (pixels[idx + 2] * (255 - a) + b * a) // 255)
                pixels[idx + 3] = min(255, old_a + a)

    def fill_circle(cx, cy, radius, r, g, b, a=255):
        for y in range(max(0, cy - radius), min(size, cy + radius + 1)):
            for x in range(max(0, cx - radius), min(size, cx + radius + 1)):
                dx, dy = x - cx, y - cy
                dist = (dx * dx + dy * dy) ** 0.5
                if dist <= radius:
                    # Anti-alias edge
                    edge_a = min(255, int(a * max(0, min(1, radius - dist + 0.5))))
                    set_pixel(x, y, r, g, b, edge_a)

    def draw_thick_line(x0, y0, x1, y1, thickness, r, g, b, a=255):
        import math
        length = max(1, int(((x1-x0)**2 + (y1-y0)**2)**0.5))
        for i in range(length + 1):
            t = i / length
            px = x0 + (x1 - x0) * t
            py = y0 + (y1 - y0) * t
            for dy in range(-thickness, thickness + 1):
                for dx in range(-thickness, thickness + 1):
                    if dx*dx + dy*dy <= thickness*thickness:
                        set_pixel(int(px + dx), int(py + dy), r, g, b, a)

    # Background circle (dark blue)
    fill_circle(cx, cy, r, 22, 27, 44)
    # Inner glow
    fill_circle(cx, cy, r - 8, 28, 35, 55)

    # Parabolic dish
    dish_cx = cx
    dish_top = int(cy - r * 0.55)
    dish_bottom = int(cy + r * 0.05)
    dish_width = int(r * 0.75)

    import math
    for x_off in range(-dish_width, dish_width + 1):
        # Parabola: y = a*x^2
        norm_x = x_off / dish_width
        curve_y = int(dish_top + (dish_bottom - dish_top) * norm_x * norm_x)
        px = dish_cx + x_off
        # Draw thick curve
        for t in range(-3, 4):
            set_pixel(px, curve_y + t, 88, 166, 255)  # Blue dish

    # Feed horn (line from center down)
    feed_top = dish_top + (dish_bottom - dish_top) // 2 - 10
    feed_bottom = int(cy + r * 0.35)
    draw_thick_line(cx, feed_top, cx, feed_bottom, 3, 160, 200, 255)

    # Base stand
    base_y = int(cy + r * 0.35)
    base_w = int(r * 0.3)
    draw_thick_line(cx - base_w, base_y + 20, cx + base_w, base_y + 20, 3, 100, 140, 200)
    draw_thick_line(cx, base_y, cx - base_w, base_y + 20, 2, 100, 140, 200)
    draw_thick_line(cx, base_y, cx + base_w, base_y + 20, 2, 100, 140, 200)

    # "MRAC" text at bottom (simple pixel font)
    text_y = int(cy + r * 0.6)
    # Simple block letters
    letters = {
        'M': [(0,0),(0,6),(1,1),(2,2),(3,1),(4,0),(4,6)],
        'R': [(0,0),(0,6),(0,0),(3,0),(4,1),(4,2),(3,3),(0,3),(2,3),(4,6)],
        'A': [(0,6),(2,0),(4,6),(1,3),(3,3)],
        'C': [(4,0),(1,0),(0,1),(0,5),(1,6),(4,6)],
    }

    # Draw "MRAC" with simple lines
    text_x_start = cx - 35
    scale = 4
    for i, ch in enumerate('MRAC'):
        bx = text_x_start + i * 20
        by = text_y
        if ch == 'M':
            draw_thick_line(bx, by+24, bx, by, 2, 63, 185, 80)
            draw_thick_line(bx, by, bx+8, by+12, 2, 63, 185, 80)
            draw_thick_line(bx+8, by+12, bx+16, by, 2, 63, 185, 80)
            draw_thick_line(bx+16, by, bx+16, by+24, 2, 63, 185, 80)
        elif ch == 'R':
            draw_thick_line(bx, by, bx, by+24, 2, 63, 185, 80)
            draw_thick_line(bx, by, bx+12, by, 2, 63, 185, 80)
            draw_thick_line(bx+12, by, bx+14, by+5, 2, 63, 185, 80)
            draw_thick_line(bx+14, by+5, bx+12, by+11, 2, 63, 185, 80)
            draw_thick_line(bx+12, by+11, bx, by+11, 2, 63, 185, 80)
            draw_thick_line(bx+8, by+11, bx+16, by+24, 2, 63, 185, 80)
        elif ch == 'A':
            draw_thick_line(bx, by+24, bx+8, by, 2, 63, 185, 80)
            draw_thick_line(bx+8, by, bx+16, by+24, 2, 63, 185, 80)
            draw_thick_line(bx+4, by+14, bx+12, by+14, 2, 63, 185, 80)
        elif ch == 'C':
            draw_thick_line(bx+14, by+2, bx+4, by, 2, 63, 185, 80)
            draw_thick_line(bx+4, by, bx, by+4, 2, 63, 185, 80)
            draw_thick_line(bx, by+4, bx, by+20, 2, 63, 185, 80)
            draw_thick_line(bx, by+20, bx+4, by+24, 2, 63, 185, 80)
            draw_thick_line(bx+4, by+24, bx+14, by+22, 2, 63, 185, 80)

    # Border ring
    for angle_deg in range(360):
        import math
        angle = math.radians(angle_deg)
        for t in range(3):
            px = int(cx + (r - t) * math.cos(angle))
            py = int(cy + (r - t) * math.sin(angle))
            set_pixel(px, py, 88, 166, 255, 200)

    return pixels


def create_ico(png_data_list, output_path):
    """Create ICO file from list of (size, png_bytes) tuples."""
    num_images = len(png_data_list)

    # ICO header
    header = struct.pack('<HHH', 0, 1, num_images)

    # Calculate offsets
    dir_size = 16 * num_images
    offset = 6 + dir_size

    dir_entries = b''
    image_data = b''

    for size, png_bytes in png_data_list:
        w = size if size < 256 else 0
        h = size if size < 256 else 0
        dir_entries += struct.pack('<BBBBHHII',
                                   w, h, 0, 0, 1, 32,
                                   len(png_bytes), offset)
        image_data += png_bytes
        offset += len(png_bytes)

    with open(output_path, 'wb') as f:
        f.write(header + dir_entries + image_data)


def main():
    icon_path = os.path.join(os.path.dirname(__file__), 'mrac_icon.ico')

    sizes = [256, 128, 64, 48, 32, 16]
    png_list = []

    for size in sizes:
        print(f"  Generating {size}x{size} icon...")
        pixels = draw_icon(size)
        png_bytes = create_png(size, size, pixels)
        png_list.append((size, png_bytes))

    create_ico(png_list, icon_path)
    print(f"  Icon saved: {icon_path}")
    return icon_path


if __name__ == '__main__':
    main()
