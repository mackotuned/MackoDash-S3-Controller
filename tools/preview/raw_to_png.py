"""Converts preview .raw frames (byte-swapped RGB565) to PNGs plus a contact sheet.

usage: raw_to_png.py <dir> [scale]
"""
import glob
import os
import sys

from PIL import Image, ImageDraw

SIZE = 240


def load(path: str) -> Image.Image:
    data = open(path, "rb").read()
    img = Image.new("RGB", (SIZE, SIZE))
    px = img.load()
    for i in range(SIZE * SIZE):
        v = (data[2 * i] << 8) | data[2 * i + 1]
        r, g, b = (v >> 11) & 0x1F, (v >> 5) & 0x3F, v & 0x1F
        px[i % SIZE, i // SIZE] = (r * 255 // 31, g * 255 // 63, b * 255 // 31)
    return img


def round_mask(img: Image.Image, scale: int) -> Image.Image:
    big = img.resize((SIZE * scale, SIZE * scale), Image.NEAREST)
    out = Image.new("RGB", big.size, (40, 40, 44))
    mask = Image.new("L", big.size, 0)
    ImageDraw.Draw(mask).ellipse((0, 0, big.size[0] - 1, big.size[1] - 1), fill=255)
    out.paste(big, (0, 0), mask)
    return out


def main() -> None:
    folder = sys.argv[1]
    scale = int(sys.argv[2]) if len(sys.argv) > 2 else 2
    frames = [round_mask(load(p), scale) for p in sorted(glob.glob(os.path.join(folder, "*.raw")))]
    cols = 6
    tile = SIZE * scale + 16
    rows = (len(frames) + cols - 1) // cols
    sheet = Image.new("RGB", (cols * tile + 16, rows * tile + 16), (40, 40, 44))
    for i, frame in enumerate(frames):
        frame.save(os.path.join(folder, f"{i:02d}.png"))
        sheet.paste(frame, (16 + (i % cols) * tile, 16 + (i // cols) * tile))
    sheet.save(os.path.join(folder, "sheet.png"))
    print(f"{len(frames)} frames -> sheet.png")


if __name__ == "__main__":
    main()
