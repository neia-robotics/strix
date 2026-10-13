"""Shared physical-size PNG generation for web and CLI calibration boards."""
import cv2
import struct
import zlib

def render_board_png(board, dpi=300):
    if not 36 <= dpi <= 600:
        raise ValueError("dpi must be between 36 and 600")
    cols, rows = board.getChessboardSize()
    px = max(1, round(board.getSquareLength() / 0.0254 * dpi))
    if (cols * px + px) * (rows * px + px) > 100_000_000:
        raise ValueError("board image exceeds 100 million pixels; lower DPI or board size")
    img = board.generateImage((cols * px + 2 * (px // 2), rows * px + 2 * (px // 2)), marginSize=px // 2, borderBits=1)
    ok, buf = cv2.imencode(".png", img)
    data = buf.tobytes()
    # Insert a pHYs chunk (pixels per meter) right after IHDR so viewers know the DPI.
    ppm = round(dpi / 0.0254)
    body = b"pHYs" + struct.pack(">IIB", ppm, ppm, 1)
    chunk = struct.pack(">I", 9) + body + struct.pack(">I", zlib.crc32(body) & 0xFFFFFFFF)
    ihdr_end = 8 + 8 + 13 + 4
    return data[:ihdr_end] + chunk + data[ihdr_end:]

