"""
plot.py

This python module provides a variety of plotting utilities used in different scripts.
"""


import os
import sys
import numpy as np
import gtsam

from comparisons import OutlierType, calc_outlier_type

# OVERHEAD TO ACCESS IRL STUFF
script_dir = os.path.dirname(__file__)
irl_dir = os.path.join(script_dir, "..", "irl", "python")
sys.path.append(irl_dir)
import irl_parsing
import irl_types



def set_axes_equal(ax) -> None:
    """
    Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.
    Args:
        ax: the axes
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])

    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))

    ax.set_xlim3d([origin[0] - radius, origin[0] + radius])
    ax.set_ylim3d([origin[1] - radius, origin[1] + radius])
    ax.set_zlim3d([origin[2] - radius, origin[2] + radius])

"""
 ######   #######  ##        #######  ########   ######
##    ## ##     ## ##       ##     ## ##     ## ##    ##
##       ##     ## ##       ##     ## ##     ## ##
##       ##     ## ##       ##     ## ########   ######
##       ##     ## ##       ##     ## ##   ##         ##
##    ## ##     ## ##       ##     ## ##    ##  ##    ##
 ######   #######  ########  #######  ##     ##  ######
"""
CORRECT_ODOM_COLOR = "black"
WRONG_ODOM_COLOR = "darkred"

TRUE_POSITIVE_LOOP_COLOR = "cornflowerblue"
FALSE_POSITIVE_LOOP_COLOR = "red"
TRUE_NEGATIVE_LOOP_COLOR = "grey"
FALSE_NEGATIVE_LOOP_COLOR = "darkgreen"


def get_loop_color(t, entry, mode):
    if t == OutlierType.TRUE_POSITIVE:
        return TRUE_POSITIVE_LOOP_COLOR, entry.measurements[mode].pose_b_key, ""
    elif t == OutlierType.FALSE_POSITIVE:
        return FALSE_POSITIVE_LOOP_COLOR, entry.measurements[mode].pose_b_key, "o"
    elif t == OutlierType.TRUE_NEGATIVE:
        return TRUE_NEGATIVE_LOOP_COLOR, entry.measurements[0].pose_b_key, ""
    elif t == OutlierType.FALSE_NEGATIVE:
        return (
            FALSE_NEGATIVE_LOOP_COLOR,
            entry.measurements[entry.correct_mode].pose_b_key,
            "o",
        )


"""
########  ##        #######  ######## #### ##    ##  ######
##     ## ##       ##     ##    ##     ##  ###   ## ##    ##
##     ## ##       ##     ##    ##     ##  ####  ## ##
########  ##       ##     ##    ##     ##  ## ## ## ##   ####
##        ##       ##     ##    ##     ##  ##  #### ##    ##
##        ##       ##     ##    ##     ##  ##   ### ##    ##
##        ########  #######     ##    #### ##    ##  ######
"""


def plot_2d_gt_traj(ax, values, lin, **kwargs):
    if lin == "nonlinear":
        poses = gtsam.utilities.allPose2s(values)
        x, y = [], []
        for key in poses.keys():
            pose = poses.atPose2(key)
            x.append(pose.x())
            y.append(pose.y())
        ax.plot(x, y, **kwargs)
    else:
        points = gtsam.utilities.extractPoint2(values)
        ax.plot(points.T[0], points.T[1], **kwargs)


def plot_3d_traj(ax, poses, lin, **kwargs):
    if lin == "nonlinear":
        x, y, z = [], [], []
        for pose in poses:
            x.append(pose.x())
            y.append(pose.y())
            z.append(pose.z())
        ax.plot(x, y, z, **kwargs)

def plot_2d_traj(ax, poses, lin, **kwargs):
    if lin == "nonlinear":
        x, y = [], []
        for pose in poses:
            x.append(pose.x())
            y.append(pose.y())
        ax.plot(x, y, **kwargs)

def plot_link(ax, k1, k2, values, color, alpha, is3d, lin, marker):
    try:
        if lin == "nonlinear":
            if is3d:
                p1, p2 = values.atPose3(k1), values.atPose3(k2)
                ax.plot(
                    [p1.x(), p2.x()],
                    [p1.y(), p2.y()],
                    [p1.z(), p2.z()],
                    color=color,
                    alpha=alpha,
                    marker=marker,
                )
            else:
                p1, p2 = values.atPose2(k1), values.atPose2(k2)
                ax.plot(
                    [p1.x(), p2.x()],
                    [p1.y(), p2.y()],
                    color=color,
                    alpha=alpha,
                    marker=marker,
                )
        else:
            if is3d:
                p1, p2 = values.atPoint3(k1), values.atPoint3(k2)
                ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    [p1[2], p2[2]],
                    color=color,
                    alpha=alpha,
                    marker=marker,
                )
            else:
                p1, p2 = values.atPoint2(k1), values.atPoint2(k2)
                ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    color=color,
                    alpha=alpha,
                    marker=marker,
                )

    except Exception:
        print("WARNING could not plot link")


def plot_dot(ax, k, values, color, alpha, is3d):
    p1 = values.atPose2(k)
    ax.plot([p1.x()], [p1.y()], marker="o", color=color, alpha=alpha)


def plot_posegraph_results(ax, values, result_modes, irl_log, is3d, show_legend=True):
    """
    Plots the results from a method in 2d
    """
    for i in range(min(len(irl_log.entries), len(result_modes))):
        entry, mode = irl_log.entries[i], result_modes[i]
        if isinstance(entry, irl_types.Prior):
            pass  # TODO (dan) how to handle multimodal prior
        elif isinstance(entry, irl_types.Odometry):
            color = (
                CORRECT_ODOM_COLOR if entry.correct_mode == mode else WRONG_ODOM_COLOR
            )
            plot_link(
                ax,
                entry.start_key,
                entry.end_key,
                values,
                color, # used for ex gridworld image "gray",
                1,
                is3d,
                irl_log.header.linearity,
                "",
            )

        elif isinstance(entry, irl_types.Loop):
            color, endpoint, marker = get_loop_color(
                calc_outlier_type(entry, mode), entry, mode
            )
            plot_link(
                ax,
                entry.pose_a_key,
                endpoint,
                values,
                color,
                0.25,
                is3d,
                irl_log.header.linearity,
                marker,
            )
        else:
            raise Exception("Invalid Entry Found in Pose Graph")

    # Make empty Plots for Legend
    leg_entry = lambda c, t: ax.plot([], [], color=c, label=t, linewidth=4)
    
    # TEMP FOR DCIST SLIDES (aug 2022)
    # leg_entry(CORRECT_ODOM_COLOR, "Trajectory")
    # leg_entry(TRUE_POSITIVE_LOOP_COLOR, "Inlier Loop Closures")
    # leg_entry(TRUE_NEGATIVE_LOOP_COLOR, "Outlier Loop Closures")


    # OLD (CORRECT)
    #leg_entry(CORRECT_ODOM_COLOR, "Correct Odometry")
    #leg_entry(WRONG_ODOM_COLOR, "Incorrect Odometry")
    leg_entry(TRUE_POSITIVE_LOOP_COLOR, "True Positive Loop Closure")
    leg_entry(FALSE_POSITIVE_LOOP_COLOR, "False Positive Loop Closure")
    leg_entry(TRUE_NEGATIVE_LOOP_COLOR, "True Negative Loop Closure")
    leg_entry(FALSE_NEGATIVE_LOOP_COLOR, "False Negative Loop Closure")
    if show_legend:
        ax.legend(loc="center left", bbox_to_anchor=(1, 0.5))


def plot_initialization(ax, irl_log,c):
    poses = []
    for entry in irl_log.entries:
        if isinstance(entry, irl_types.Prior):
            poses.append(entry.measurements[0].pose)
        elif isinstance(entry, irl_types.Odometry):
            poses.append(poses[-1].compose(entry.measurements[0].pose))

    x,y=[],[]
    for pose in poses:
        x.append(pose.x())
        y.append(pose.y())

    ax.plot(x,y, "-.", color=c, alpha=1)
    

