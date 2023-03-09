import gtsam
import numpy as np
import irl_types
from gtsam.symbol_shorthand import X, L

"""
##     ##    ###    ##       ##     ## ########  ######  
##     ##   ## ##   ##       ##     ## ##       ##    ## 
##     ##  ##   ##  ##       ##     ## ##       ##       
##     ## ##     ## ##       ##     ## ######    ######  
 ##   ##  ######### ##       ##     ## ##             ## 
  ## ##   ##     ## ##       ##     ## ##       ##    ## 
   ###    ##     ## ########  #######  ########  ######  
"""


def parse_values_file(in_file):
    def parse_values_line(line):
        result = gtsam.Values()
        tokens = line.strip().split(" ")
        num_poses = 0

        i = 0
        while i < len(tokens):
            if tokens[i] == "POSE2":
                idx = int(tokens[i + 1])
                x, y, theta = map(float, tokens[i + 2 : i + 5])
                result.insert(X(idx), gtsam.Pose2(x, y, theta))
                i += 5
                num_poses += 1
            elif tokens[i] == "POSE3":
                idx = int(tokens[i + 1])
                x, y, z, rx, ry, rz = map(float, tokens[i + 2 : i + 8])
                result.insert(
                    X(idx),
                    gtsam.Pose3(gtsam.Rot3.RzRyRx(rx, ry, rz), np.array([x, y, z])),
                )
                i += 8
                num_poses += 1
            elif tokens[i] == "POINT2":
                idx = int(tokens[i + 1])
                x, y = map(float, tokens[i + 2 : i + 4])
                result.insert(X(idx), gtsam.Point2(x, y))
                i += 4
                num_poses += 1
            elif tokens[i] == "POINT3":
                idx = int(tokens[i + 1])
                x, y, z = map(float, tokens[i + 2 : i + 5])
                result.insert(X(idx), gtsam.Point2(x, y, z))
                i += 5
                num_poses += 1
            else:
                raise Exception("Unknown type found in values file")
        return (result, num_poses)

    result = []
    with open(in_file) as f:
        for line in f.readlines():
            result.append(parse_values_line(line))
    return result


def write_values_file(out_file, dim, lin, list_of_values):
    def write_values_line(f, values):
        if lin == "nonlinear":
            # Write Poses
            if dim == 3:
                poses = gtsam.utilities.allPose3s(values)
                for key in poses.keys():
                    pose = poses.atPose3(key)
                    s = gtsam.Symbol(key)
                    f.write(
                        "POSE3 {} {} ".format(
                            s.index(), irl_types.serialize_pose(dim, lin, pose)
                        )
                    )

            else:
                poses = gtsam.utilities.allPose2s(values)
                for key in poses.keys():
                    pose = poses.atPose2(key)
                    s = gtsam.Symbol(key)
                    f.write(
                        "POSE2 {} {} ".format(
                            s.index(), irl_types.serialize_pose(dim, lin, pose)
                        )
                    )
        elif lin == "linear":
            if dim == 3:
                points = gtsam.utilities.extractPoint3(values)
                for i in range(len(points)):
                    f.write(
                        "POINT3 {} {} ".format(
                            i, irl_types.serialize_pose(dim, lin, pose)
                        )
                    )
            else:
                points = gtsam.utilities.extractPoint2(values)
                for i in range(len(points)):
                    f.write(
                        "POINT2 {} {} ".format(
                            i, irl_types.serialize_pose(dim, lin, pose)
                        )
                    )

    with open(out_file, "w") as f:
        for vals in list_of_values:
            write_values_line(f, vals)
            f.write("\n")


"""
##     ##  #######  ########  ########  ######  
###   ### ##     ## ##     ## ##       ##    ## 
#### #### ##     ## ##     ## ##       ##       
## ### ## ##     ## ##     ## ######    ######  
##     ## ##     ## ##     ## ##             ## 
##     ## ##     ## ##     ## ##       ##    ## 
##     ##  #######  ########  ########  ######  
"""


def parse_modes_file(in_file):
    result = []
    with open(in_file) as f:
        for line in f.readlines():
            result.append(list(map(int, line.strip().split(" "))))
    return result


def write_modes_file(out_file, list_of_modes):
    with open(out_file, "w") as f:
        for mode_seq in list_of_modes:
            f.write(" ".join(map(str, mode_seq)))
            f.write("\n")
