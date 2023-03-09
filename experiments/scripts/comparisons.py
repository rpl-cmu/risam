""" 
This python module implements a number of methods to compute metrics on results from run-experiment.
"""
import os
import sys
from enum import Enum
from unittest import result
from enum import Enum
import gtsam
import numpy as np

script_dir = os.path.dirname(__file__)
irl_dir = os.path.join(script_dir, "..", "irl", "python")
sys.path.append(irl_dir)
import irl_parsing
import irl_types


class OutlierType(Enum):
    TRUE_POSITIVE = 1
    TRUE_NEGATIVE = 2
    FALSE_POSITIVE = 3
    FALSE_NEGATIVE = 4
    # identified correct mode as inlier but allowed other mutually exclusive modes
    PARTIAL_TRUE_POSITIVE = 5


def calc_outlier_type(entry, chosen_mode):
    if entry.correct_mode == chosen_mode:  # TRUE
        return (
            OutlierType.TRUE_NEGATIVE
            if isinstance(entry.measurements[entry.correct_mode], irl_types.NullHypo)
            else OutlierType.TRUE_POSITIVE
        )
    else:  # FALSE
        return (
            OutlierType.FALSE_NEGATIVE
            if isinstance(entry.measurements[chosen_mode], irl_types.NullHypo)
            else OutlierType.FALSE_POSITIVE
        )


def calc_precision_recall(modes, log):
    TP, TN, FP, FN = 0, 0, 0, 0

    contains_mh = False
    for mode, entry in zip(modes, log.entries):
        if entry.num_modes > 1:  # only count MH measurements
            contains_mh = True
            mtype = calc_outlier_type(entry, mode)
            if mtype == OutlierType.TRUE_POSITIVE:
                TP += 1
            elif mtype == OutlierType.TRUE_NEGATIVE:
                TN += 1
            elif mtype == OutlierType.FALSE_POSITIVE:
                FP += 1
            elif mtype == OutlierType.FALSE_NEGATIVE:
                FN += 1

    if contains_mh:
        precision = TP / (TP + FP) if (TP + FP) > 0 else -1.0
        recall = TP / (TP + FN) if (TP + FN) > 0 else -1.0
    else:
        precision = 1.0
        recall = 1.0
    return precision, recall


def umeyama_alignment(x: np.ndarray, y: np.ndarray):
    """
    Computes the least squares transform that aligns point-set y to point-set x
    cite:
    [1] S. Umeyama, "Least-squares estimation of transformation parameters between two point patterns,"
    in IEEE Transactions on Pattern Analysis and Machine Intelligence,
    vol. 13, no. 4, pp. 376-380, April 1991, doi: 10.1109/34.88573.

    x: Array of n points with dimension d (n,d)
    y: Array of n points with dimension d (n,d)
    Assumption: x[i] associates with y[i]
    """
    num_pts, dim = x.shape
    # Find mean [1, Eq. 34] [1, Eq. 35]
    mu_x, mu_y = np.mean(x, axis=0), np.mean(y, axis=0)

    # Variance [1, Eq. 36] [1, Eq. 37]
    var_x = np.mean(np.linalg.norm(x - mu_x, axis=1))
    var_y = np.mean(np.linalg.norm(y - mu_y, axis=1))

    # Covariance [1, Eq. 38]
    cov = (1.0 / num_pts) * (y - mu_y).T @ (x - mu_x)

    # SVD
    U, _, Vt = np.linalg.svd(cov)

    # Matrix S [1, Eq. 43]
    S = np.eye(dim)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[-1, -1] = -1

    # Compute Transform
    R = U @ S @ Vt  # [1, Eq. 40]
    t = mu_y - R @ mu_x  # [1, Eq. 42]

    return R, t


def align_trajectory(
    est_traj,
    ref_traj,
):
    """
    Aligns the estimated trajectory to the reference trajectory with Uemyama alignment
    ref_traj: List-of-gtsam.Pose{2/3} the reference trajectory
    est_traj: List-of-gtsam.Pose{2/3} the estimated trajectory
    align_with_scale: If true preforms Umeyama Alignment with scale correction
    """
    est_positions = []
    ref_positions = []
    for ep, rp in zip(est_traj, ref_traj):
        est_positions.append(ep.translation())
        ref_positions.append(rp.translation())
    est_positions = np.stack(est_positions)
    ref_positions = np.stack(ref_positions)
    R, t = umeyama_alignment(est_positions, ref_positions)

    if isinstance(est_traj[0], gtsam.Pose3):
        transform = gtsam.Pose3(gtsam.Rot3(R), t)
    else:
        transform = gtsam.Pose2(gtsam.Rot2(np.arccos(R[0, 0])), t)

    return [transform.compose(p) for p in est_traj]


def calc_trajectory_metrics(ref_traj, est_traj):
    """
    Compute the RMSE Absolute Trajectory Error and Relative Pose Error
    for both translation and rotation for detailed definitions see...
    https://rpg.ifi.uzh.ch/docs/IROS18_Zhang.pdf

    Params:
    ref_traj: List-of-gtsam.Pose{2/3} the reference trajectory
    est_traj: List-of-gtsam.Pose{2/3} the estimated trajectory

    We assume that each trajectory is already aligned and pose rej_traj[i] corresponds to pose est_traj[i]
    """
    ate_pos, ate_rot = [], []
    rpe_pos, rpe_rot = [], []
    for i in range(len(ref_traj) - 1):
        # Compute Absolute Traj Error
        abs_error = ref_traj[i].inverse().compose(est_traj[i])
        ate_pos.append(np.linalg.norm(abs_error.translation()))
        if isinstance(abs_error, gtsam.Pose2):
            ate_rot.append(abs_error.rotation().theta())
        else:
            ate_rot.append(abs_error.rotation().axisAngle()[1])

        # Compute Relative Pose Error
        de = est_traj[i + 1].inverse().compose(est_traj[i])
        dr = ref_traj[i + 1].inverse().compose(ref_traj[i])
        rel_error = de.inverse().compose(dr)
        rpe_pos.append(np.linalg.norm(rel_error.translation()))
        if isinstance(abs_error, gtsam.Pose2):
            rpe_rot.append(rel_error.rotation().theta())
        else:
            rpe_rot.append(rel_error.rotation().axisAngle()[1])

    # Convert to numpy
    ate_pos, ate_rot = np.array(ate_pos), np.array(ate_rot)
    rpe_pos, rpe_rot = np.array(rpe_pos), np.array(rpe_rot)

    # Generate Results (RMSE)
    results = {}
    N = ate_pos.shape[0]
    rmse = lambda x, n: np.sqrt((x ** 2).sum() / n)
    results["ate_pos_rmse"] = rmse(ate_pos, N)
    results["ate_rot_rmse"] = rmse(ate_rot, N)
    results["rpe_pos_rmse"] = rmse(rpe_pos, N - 1)
    results["rpe_rot_rmse"] = rmse(rpe_rot, N - 1)
    results["ate_pos"] = ate_pos
    results["ate_rot"] = ate_rot
    results["rpe_pos"] = rpe_pos
    results["rpe_rot"] = rpe_rot
    return results
