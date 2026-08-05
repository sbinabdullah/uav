#!/usr/bin/env python3
"""Generate ArUco marker PNGs for the sar_zone world panels.

Writes sim/models/aruco_<id>/marker.png for each id in IDS.
Dictionary: DICT_4X4_50 (small cells, robust at distance).
"""
import os
import cv2

IDS = [0, 1, 2, 3]
SIZE_PX = 400          # marker texture resolution
BORDER_BITS = 1        # white border in marker cells
OUT_DIR = os.path.join(os.path.dirname(__file__), "..", "sim", "models")

def main():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    for marker_id in IDS:
        img = cv2.aruco.generateImageMarker(aruco_dict, marker_id, SIZE_PX)
        # add a quiet zone (white border) of one marker-cell width
        cell = SIZE_PX // (4 + 2 * BORDER_BITS)
        bordered = cv2.copyMakeBorder(
            img, cell, cell, cell, cell, cv2.BORDER_CONSTANT, value=255)
        out = os.path.join(OUT_DIR, f"aruco_{marker_id}", "marker.png")
        os.makedirs(os.path.dirname(out), exist_ok=True)
        cv2.imwrite(out, bordered)
        print(f"wrote {out} (id={marker_id})")

if __name__ == "__main__":
    main()
