import os
from pathlib import Path
from typing import Union

import numpy as np

def save_pointcloud_from_ORB_SLAM(input_file: Union[Path, str], output_file: Union[Path,str] = "out.ply"):
    """Converts a comma separated list of map point coordinates into
    PLY format for viewing the generated map.

    Args:
        input_file (str or Path): Path to the input file which is expected to
        be a .csv file with the columns pos_x, pos_y, pos_z designating the
        coordinates of the points in the world reference frame.

        output_file (str or Path): Path to the output .ply file, format is
        described here: https://paulbourke.net/dataformats/ply/
    """

    coords = np.genfromtxt(input_file, delimiter=", ", skip_header=1)

    x = coords[:, 0]
    y = coords[:, 1]
    z = coords[:, 2]

    ply_header = 'ply\n' \
                'format ascii 1.0\n' \
                'element vertex %d\n' \
                'property float x\n' \
                'property float y\n' \
                'property float z\n' \
                'end_header' % x.shape[0]

    np.savetxt(output_file, np.column_stack((x, y, z)), fmt='%f %f %f', header=ply_header, comments='')

def save_trajectory_from_ORB_SLAM(input_file: Union[Path, str], output_file: Union[Path,str] = "out_trajectory.ply"):
    """Converts the saved trajectory file from ORB-SLAM3 to a point cloud to then
    show alongside the mapped cloud.

    The input file is expected to be in the format:
    timestamp tx ty tz qx qy qz qw
    Where tx, ty, tz are the translation components and qx, qy, qz, qw form a quaternion.
    """
    x = []
    y = []
    z = []

    with open(input_file, "r") as file:
        lines = file.readlines()

    for line in lines:
        cols = line.strip().split()
        if len(cols) != 8:
            print(f"Skipping invalid line: {line.strip()}")
            continue

        x.append(float(cols[1]))  # tx
        y.append(float(cols[2]))  # ty
        z.append(float(cols[3]))  # tz

    # Each trajectory point is shown as a "point" in the "point cloud" which is the trajectory
    x = np.array(x)
    y = np.array(y)
    z = np.array(z)

    # RGB values for each point on the trajectory, set to be light green
    r = np.ones_like(x) * 144
    g = np.ones_like(x) * 238
    b = np.ones_like(x) * 144

    ply_header = 'ply\n' \
                'format ascii 1.0\n' \
                f'element vertex {len(x)}\n' \
                'property float x\n' \
                'property float y\n' \
                'property float z\n' \
                'property uchar red\n' \
                'property uchar green\n' \
                'property uchar blue\n' \
                'end_header'

    np.savetxt(output_file, np.column_stack((x, y, z, r, g, b)), fmt='%f %f %f %d %d %d', header=ply_header, comments='')



if __name__ == "__main__":
    input_file = "/home/pi/ORB_SLAM3/ref_pointcloud.txt" # ReferenceMapPoints seem to work better, use that file
    output_file = "/home/pi/ORB_SLAM3/output.ply"

    input_trajectory = "/home/pi/ORB_SLAM3/KeyFrameTrajectory.txt"
    output_trajectory = "/home/pi/ORB_SLAM3/output_trajectory.ply"

    save_pointcloud_from_ORB_SLAM(input_file, output_file)
    save_trajectory_from_ORB_SLAM(input_trajectory, output_trajectory)