#!/usr/bin/env python3
"""Generate Gazebo models for ArUco victim markers.

For each ID this writes <out>/aruco_<id>/:
  model.config
  model.sdf                              thin static box
  materials/textures/aruco_<id>.png      marker + white quiet zone
  materials/scripts/aruco_<id>.material  OGRE material that applies the PNG

marker_length is the side of the BLACK square, the value
cv2.aruco.estimatePoseSingleMarkers needs. The white border around it is
extra, so the tile is marker_length + 2 * border wide.
"""
import argparse
import os

import cv2

PX_PER_CELL = 100

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>aruco_{id}</name>
  <version>1.0</version>
  <sdf version="1.6">model.sdf</sdf>
  <description>ArUco {dict_name} id {id}, black square {length} m</description>
</model>
"""

MODEL_SDF = """<?xml version="1.0"?>
<sdf version="1.6">
  <model name="aruco_{id}">
    <static>true</static>
    <link name="link">
      <collision name="collision">
        <geometry><box><size>{thickness} {tile} {tile}</size></box></geometry>
      </collision>
      <visual name="visual">
        <geometry><box><size>{thickness} {tile} {tile}</size></box></geometry>
        <material>
          <script>
            <uri>model://aruco_{id}/materials/scripts</uri>
            <uri>model://aruco_{id}/materials/textures</uri>
            <name>ArucoMarker/Id{id}</name>
          </script>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""

MATERIAL = """material ArucoMarker/Id{id}
{{
  technique
  {{
    pass
    {{
      lighting off
      texture_unit
      {{
        texture aruco_{id}.png
        filtering none
      }}
    }}
  }}
}}
"""


def make_texture(dictionary, marker_id, marker_length, border):
    cells = dictionary.markerSize + 2          # data bits + 1-cell black border each side
    marker_px = cells * PX_PER_CELL
    img = cv2.aruco.drawMarker(dictionary, marker_id, marker_px)
    border_px = int(round(marker_px * border / marker_length))
    return cv2.copyMakeBorder(img, border_px, border_px, border_px, border_px,
                              cv2.BORDER_CONSTANT, value=255)


def main():
    default_out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'models')
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--ids', type=int, nargs='+', default=[0, 1, 2, 3, 4, 5])
    p.add_argument('--dict', default='DICT_4X4_50')
    p.add_argument('--marker-length', type=float, default=0.20, help='black square side [m]')
    p.add_argument('--border', type=float, default=0.05, help='white quiet zone per side [m]')
    p.add_argument('--thickness', type=float, default=0.01, help='box depth [m]')
    p.add_argument('--out', default=default_out)
    args = p.parse_args()

    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, args.dict))
    tile = args.marker_length + 2 * args.border

    for marker_id in args.ids:
        model_dir = os.path.join(args.out, f'aruco_{marker_id}')
        tex_dir = os.path.join(model_dir, 'materials', 'textures')
        script_dir = os.path.join(model_dir, 'materials', 'scripts')
        os.makedirs(tex_dir, exist_ok=True)
        os.makedirs(script_dir, exist_ok=True)

        cv2.imwrite(os.path.join(tex_dir, f'aruco_{marker_id}.png'),
                    make_texture(dictionary, marker_id, args.marker_length, args.border))
        fields = dict(id=marker_id, dict_name=args.dict, length=args.marker_length,
                      thickness=args.thickness, tile=round(tile, 4))
        with open(os.path.join(model_dir, 'model.config'), 'w') as f:
            f.write(MODEL_CONFIG.format(**fields))
        with open(os.path.join(model_dir, 'model.sdf'), 'w') as f:
            f.write(MODEL_SDF.format(**fields))
        with open(os.path.join(script_dir, f'aruco_{marker_id}.material'), 'w') as f:
            f.write(MATERIAL.format(**fields))
        print(f'aruco_{marker_id}: tile {tile:.2f} m, black square {args.marker_length:.2f} m')

    print(f'Wrote {len(args.ids)} models to {os.path.abspath(args.out)}')
    print(f'Remember: detection.yaml marker_length must be {args.marker_length}')


if __name__ == '__main__':
    main()
