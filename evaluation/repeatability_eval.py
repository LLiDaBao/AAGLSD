import math
import numpy as np
import os
import argparse
from utils import *
import sys


def proj_points_to_line(ref_segments, points):
    r""" Project points to reference segments.
    
        Shape:
            - ref_segments: `(N, 4)`.
            - points: `(M, 2)`.

    """
    # line_vec_ref: (N, M, 2)
    line_vec_ref = (ref_segments[:, 2:] - ref_segments[:, :2])[:, None, :]
    line_vec_ref = np.repeat(line_vec_ref, points.shape[0], axis=1)

    # mid_pt_vec: (N, M, 2)
    mid_pt_vec = points[None, :, :] - ref_segments[:, None, :2]

    # shape: (N, M, 2)
    proj_points = ref_segments[:, None, :2] + line_vec_ref * \
        np.sum(line_vec_ref * mid_pt_vec, axis=-1, keepdims=True) / \
        np.sum(line_vec_ref * line_vec_ref, axis=-1, keepdims=True)
    
    return proj_points



def proj_points_to_line_(ref_segments, points):
    r""" Project points to reference segments.
    
        Shape:
            - ref_segments: `(N, 4)`.
            - points: `(N, 2)`.

    """
    line_vec_ref = ref_segments[:, 2:] - ref_segments[:, :2]
    mid_pt_vec = points - ref_segments[:, :2]

    # shape: (N, 2)
    proj_points = ref_segments[:, :2] + line_vec_ref * \
        np.sum(line_vec_ref * mid_pt_vec, axis=-1, keepdims=True) / \
        np.sum(line_vec_ref * line_vec_ref, axis=-1, keepdims=True)
    
    return proj_points



def center_perpendicular_dists(ref_segments, pred_segments):
    r""" Perpendicular distance between centers in the orthogonal direction 
    to reference segment. The details could see in Linelet.

        Shape:
            - ref_segments: `(N, 4)`.
            - pred_segments: `(M, 4)`.

    """
    N, M = ref_segments.shape[0], pred_segments.shape[0]

    # shape: (M, 2)
    mid_points_pred = (pred_segments[:, :2] + pred_segments[:, 2:]) * 0.5

    # shape: (N, M, 2)
    proj_points = proj_points_to_line(ref_segments, mid_points_pred)
    #print(proj_points)

    # shape: (N, M)
    return np.sqrt(np.sum((proj_points - mid_points_pred[None, :, :]) ** 2, axis=-1))



def line_angle_diff(segments0, segments1):
    # segments0: (N, 4), segments1: (M, 4)
    N, M = segments0.shape[0], segments1.shape[0]

    # vec0: (N, 2), vec1: (M, 2)
    vec0 = segments0[:, 2:] - segments0[:, :2]
    vec1 = segments1[:, 2:] - segments1[:, :2]

    # line_ang0: (N, ), line_ang1: (M, )
    eps = 1e-6
    line_ang0 = np.arctan(vec0[:, 1] / (vec0[:, 0] + eps))
    line_ang1 = np.arctan(vec1[:, 1] / (vec1[:, 0] + eps))
    #print(line_ang0.shape, line_ang1.shape)

    # shape: (N, M)
    ang_diff = line_ang0[:, None] - line_ang1[None, :]
    #print(ang_diff.shape)

    return ang_diff



def intersec_length(ref_segments, pred_segments):
    r""" Project predicted segments to reference segments, and calculate the
    intersectioin raito between reference segments and projected segments.

        Shape:
            - ref_segments: `(N, 4)`.
            - pred_segments: `(N, 4)`.
    
    """
    N, M = ref_segments.shape[0], pred_segments.shape[0]
    assert N == M

    def dot(vec0, vec1):
        return np.sum(vec0 * vec1, axis=-1)
    
    def vec_len(pts0, pts1):
        return np.sqrt(np.sum((pts0 - pts1) ** 2, axis=-1))

    # 
    proj_segments = np.concatenate([
        proj_points_to_line_(ref_segments, pred_segments[:, :2]),
        proj_points_to_line_(ref_segments, pred_segments[:, 2:])
    ], axis=-1)
    #print("Project segments shape:", proj_segments.shape)

    # shape: (N, )
    lengths_ref = np.sqrt(
        np.sum((ref_segments[:, :2] - ref_segments[:, 2:]) ** 2, axis=-1))
    lenghts_proj = np.sqrt(
        np.sum((proj_segments[:, :2] - proj_segments[:, 2:]) ** 2, axis=-1))
    #print("Length shape:", lengths_ref.shape, lenghts_proj.shape)

    # output
    inter_lens = [0.0] * N

    # vector
    vec_proj0_to_ref0 = ref_segments[:, :2] - proj_segments[:, :2]
    vec_proj0_to_ref1 = ref_segments[:, 2:] - proj_segments[:, :2] 
    vec_proj1_to_ref0 = ref_segments[:, :2] - proj_segments[:, 2:] 
    vec_proj1_to_ref1 = ref_segments[:, 2:] - proj_segments[:, 2:]
    
    vec_ref0_to_proj0 = proj_segments[:, :2] - ref_segments[:, :2]
    vec_ref0_to_proj1 = proj_segments[:, 2:] - ref_segments[:, :2]
    vec_ref1_to_proj0 = proj_segments[:, :2] - ref_segments[:, 2:]
    vec_ref1_to_proj1 = proj_segments[:, 2:] - ref_segments[:, 2:]

    # if result <= 0, dot_pt#_seg means pt# is in seg
    dot_proj0_ref = dot(vec_proj0_to_ref0, vec_proj0_to_ref1) <= 0
    dot_proj1_ref = dot(vec_proj1_to_ref0, vec_proj1_to_ref1) <= 0
    dot_ref0_proj = dot(vec_ref0_to_proj0, vec_ref0_to_proj1) <= 0
    dot_ref1_proj = dot(vec_ref1_to_proj0, vec_ref1_to_proj1) <= 0

    for ind in range(N):
        if lenghts_proj[ind] < 1:
            continue
        
        # reference segment is in projected segment 
        if dot_ref0_proj[ind] and dot_ref1_proj[ind]:
            intersec_len = lengths_ref[ind]

        # projected segment is in reference segment
        elif dot_proj0_ref[ind] and dot_proj1_ref[ind]:
            intersec_len = lenghts_proj[ind]

        # proj0 in, proj1 out, ref0 in, ref1 out
        elif dot_proj0_ref[ind] and not dot_proj1_ref[ind] and \
            dot_ref0_proj[ind] and not dot_ref1_proj[ind]:
            intersec_len = vec_len(proj_segments[ind, :2], ref_segments[ind, :2])

        # proj0 out, proj1 in, ref0 in, ref1 out
        elif not dot_proj0_ref[ind] and dot_proj1_ref[ind] and \
            dot_ref0_proj[ind] and not dot_ref1_proj[ind]:
            intersec_len = vec_len(proj_segments[ind, 2:], ref_segments[ind, :2])

        # proj0 in, proj1 out, ref0 out, ref1 in
        elif dot_proj0_ref[ind] and not dot_proj1_ref[ind] and not \
            dot_ref0_proj[ind] and dot_ref1_proj[ind]:
            intersec_len = vec_len(proj_segments[ind, :2], ref_segments[ind, 2:])

        # proj0 out, proj1 in, ref0 out, ref1 in
        elif not dot_proj0_ref[ind] and dot_proj1_ref[ind] and not \
            dot_ref0_proj[ind] and dot_ref1_proj[ind]:
            intersec_len = vec_len(proj_segments[ind, 2:], ref_segments[ind, 2:])

        # no intersection
        else:
            intersec_len = 0.0

        inter_lens[ind] = intersec_len

    # shape: (N, )
    return np.array(inter_lens, dtype=np.float32)
        


def repeatability(
        ref_segments, 
        pred_segments, 
        dist_thresh=1.5, 
        ang_thresh=5.0 * math.pi / 180.0, 
        area_thresh=0.5,
        pixelwise=False
    ):
    r""" Evaluate repeatability, it is defined as `rep = n_m / 2 * (1 / n_r + 1 / n_t)`. 
    `n_m` is the number of matched line segments, `n_r` is the number of line segments in 
    reference image, and `n_t` is the number of line segments in test image.

        Args:
            ref_segments: line segments in reference image.
            matched_segments: line segments which was generated by bipartite match.
            dist_thresh: threshold for center distance between 2 line segments.
            ang_thresh: threshold for angle difference between 2 line segments.
            area_thresh: threshold for intersection ratio between 2 line segments.

        Shape:
            - ref_segments: `(N, 4)`.
            - pred_segments: `(M, 4)`.

    """
    num_ref, num_pred = ref_segments.shape[0], pred_segments.shape[0]

    if num_ref == 0 or num_pred == 0:
        raise Exception("Segment pairs dimension got wrong!")

    # shape: (N, M)
    center_dists0 = center_perpendicular_dists(ref_segments, pred_segments)
    center_dists1 = center_perpendicular_dists(pred_segments, ref_segments).T

    # shape: (N, M)
    ang_diffs = line_angle_diff(ref_segments, pred_segments)

    # predicted segments' indices for every ground-truth segments,
    # which center_dist < thresh and angle_diff < thresh
    candidate_inds = (center_dists0 <= dist_thresh) & \
        (center_dists1 <= dist_thresh) & \
        (ang_diffs <= ang_thresh)

    #print(matched_indices.shape)


    # evaluate repeatability
    ref_lens = np.sqrt(
        np.sum((ref_segments[:, :2] - ref_segments[:, 2:]) ** 2, axis=-1))
    pred_lens = np.sqrt(
        np.sum((pred_segments[:, :2] - pred_segments[:, 2:]) ** 2, axis=-1))

    # true positive in instance-level and pixel-level
    num_tp_inst = 0
    num_tp_pixel = 0

    for ind in range(num_ref):
        num_candidates = np.count_nonzero(candidate_inds[ind])

        if num_candidates == 0:
            continue

        _ref_segments = np.repeat(
            ref_segments[ind][None, :], num_candidates, axis=0)

        _pred_segments = pred_segments[candidate_inds[ind]]

        # for validation
        _ref_lens = np.repeat(ref_lens[ind], num_candidates, axis=0)
        _pred_lens = pred_lens[candidate_inds[ind]]
        
        #print(_pred_segments.shape, _ref_segments.shape)

        # shape: (num_pred, )
        intersec = intersec_length(_ref_segments, _pred_segments)

        # cross calculate, in order to avoid over-connection
        valid_inds = ((intersec / _ref_lens) >= area_thresh) & \
            ((intersec / _pred_lens) >= area_thresh)
        
        if np.count_nonzero(valid_inds) > 0:
            num_tp_inst += 1
            num_tp_pixel += (intersec[valid_inds]).sum()


    # pixel-wise repeatability
    if pixelwise:
        num_ref = ref_lens.sum()
        num_pred = pred_lens.sum()
        return 0.5 * num_tp_pixel * (1.0 / num_ref + 1.0 / num_pred)

    # instance-wise repeatability
    return 0.5 * num_tp_inst * (1.0 / num_ref + 1.0 / num_pred)



if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument("--dist_thresh", type=float, nargs='?', default=3.0, const=3.0, help="Distance threshold between line segments.")
    parser.add_argument("--ang_thresh", type=float, nargs='?', default=3.0 * math.pi / 180.0, const=3.0 * math.pi / 180.0, help="Angle difference threshold between line segments.")
    parser.add_argument("--area_thresh", type=float, nargs='?', default=0.75, const=0.75, help="Intersection area difference threshold between line segments.")
    parser.add_argument("--pixelwise", action="store_true")
    parser.add_argument("--dir_hpatches", type=str, default="your directory", help="Directory to Hpatches dataset.")
    parser.add_argument("--dir_pred", type=str, default="your directory", help="Directory to the predictions.")

    args = parser.parse_args()

    folder_names = os.listdir(args.dir_hpatches)

    reps = []   # repeatability

    # Load predicted segments
    for folder_name in folder_names:
        mean_rep = 0.0

        ref_segments = load_csv_data(
            os.path.join(args.dir_pred, folder_name, "1.csv"))
        
        for i in range(2, 7):
            homo_mat = load_homography_data(
                os.path.join(args.dir_hpatches, folder_name, f"H_1_{i}"))

            # apply homography to reference segments
            homo_segments = transform_ref_segments(ref_segments, homo_mat)

            # load predicted segments
            pred_segments = load_csv_data(
                os.path.join(args.dir_pred, folder_name, f"{i}.csv"))

            # reps
            rep = repeatability(
                homo_segments, pred_segments, 
                dist_thresh=args.dist_thresh, 
                ang_thresh=args.ang_thresh, 
                area_thresh=args.area_thresh,
                pixelwise=args.pixelwise
            )
            mean_rep += rep

        reps.append(mean_rep / 5.0)
        print(f"{folder_name}: {reps[-1]}")
        if len(reps) > 57: break

    reps = np.array(reps, dtype=np.float32)
    light_reps = reps[:57]
    view_reps = reps[57:]

    print(f"Mean Repeatability:{reps.mean()}")
    print(f"Mean Light Repeatability:{light_reps.mean()}")
    print(f"Mean View Repeatability:{view_reps.mean()}")