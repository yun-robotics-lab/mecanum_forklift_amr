#!/usr/bin/env python3
"""
Generate Gazebo ArUco marker models (texture + SDF).

Usage: generate_marker_models.py OUT_DIR IDS...

Each model `aruco_marker_<id>` is a thin white board with the marker on its local +x face
(marker x-right/up convention: the textured plane normal is local +x, image up is local +z).
Dictionary DICT_4X4_50; marker side 0.10 m inside a white quiet zone (board 0.13 m).
"""
import os
import sys

import cv2

from mecanum_forklift_vision.pose_core import draw_marker_image

DICT = 'DICT_4X4_50'
MARKER = 0.10
BOARD = 0.13
THICK = 0.008
PX = 400          # marker pixels
BORDER = 100      # white quiet-zone pixels each side -> plane side = MARKER * 600 / 400

SDF = """<?xml version="1.0"?>
<sdf version="1.8">
  <model name="aruco_marker_{i}">
    <static>true</static>
    <link name="link">
      <visual name="board">
        <pose>{bx} 0 0 0 0 0</pose>
        <geometry><box><size>{t} {b} {b}</size></box></geometry>
        <material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse></material>
      </visual>
      <visual name="marker">
        <pose>0.0005 0 0 1.570796 0 1.570796</pose>
        <geometry><plane><normal>0 0 1</normal><size>{p} {p}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse><specular>0 0 0 1</specular>
          <pbr><metal>
            <albedo_map>model://aruco_marker_{i}/materials/textures/marker_{i}.png</albedo_map>
          <roughness>1</roughness><metalness>0</metalness></metal></pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

CONFIG = """<?xml version="1.0"?>
<model>
  <name>aruco_marker_{i}</name>
  <version>1.0</version>
  <sdf version="1.8">model.sdf</sdf>
  <description>ArUco DICT_4X4_50 id {i}, 0.10 m marker, 0.13 m board, face +x</description>
</model>
"""


def main():
    out = sys.argv[1]
    for i in (int(a) for a in sys.argv[2:]):
        d = os.path.join(out, f'aruco_marker_{i}')
        tex = os.path.join(d, 'materials', 'textures')
        os.makedirs(tex, exist_ok=True)
        img = draw_marker_image(DICT, i, PX)
        img = cv2.copyMakeBorder(img, BORDER, BORDER, BORDER, BORDER,
                                 cv2.BORDER_CONSTANT, value=255)
        cv2.imwrite(os.path.join(tex, f'marker_{i}.png'), img)
        plane = MARKER * (PX + 2 * BORDER) / PX
        with open(os.path.join(d, 'model.sdf'), 'w') as f:
            f.write(SDF.format(i=i, bx=-THICK / 2, t=THICK, b=BOARD, p=plane))
        with open(os.path.join(d, 'model.config'), 'w') as f:
            f.write(CONFIG.format(i=i))


if __name__ == '__main__':
    main()
