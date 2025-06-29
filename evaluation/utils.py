import os
import numpy as np
import math



def list_csv_files(dir):
    r""" Return the list of csv files."""
    filenames = os.listdir(dir)
    
    return [os.path.join(dir, filename) for filename in filenames]



def list_hpatches_files(hpathces_sequence_dir):
    r""" Return the list of lists of hpatches image files and homography files.
    
    Return:
        - image_lists: list of names of image files, suffix is .ppm
        - homo_lists: list of names of homography files.
    
    """
    if not os.path.exists(hpathces_sequence_dir):
        raise Exception("Directory doesn't exist.")
    
    file_dirs = os.listdir(hpathces_sequence_dir)
    
    if len(file_dirs) == 0:
        raise Exception("Empty directory.")
    
    file_dirs = [os.path.join(
        hpathces_sequence_dir, file_dir) for file_dir in file_dirs]
    
    image_lists, homo_lists = [], []
    
    for file_dir in file_dirs:
        file_names = os.listdir(file_dir)
        image_list, homo_list = [], []
        for file_name in file_names:
            name, ext = os.path.splitext(file_name)
            if ext == '.ppm':
                image_list.append(os.path.join(file_dir, file_name))
            else:
                homo_list.append(os.path.join(file_dir, file_name))
        
        # sort file names to keep sequence
        image_list.sort()
        homo_list.sort()
        image_lists.append(image_list)
        homo_lists.append(homo_list)

    return image_lists, homo_lists



def load_csv_data(csv_path):
    r""" Reading line segments like '(x0, y0, x1, y1)' from disk.

    Args:
        csv_path: string path of csv file.
        
    Returns:
        numpy array of line segments with float32 data type.

    """
    if not os.path.exists(csv_path):
        raise Exception(f"{csv_path} doesn't exist.")
    
    _, ext = os.path.splitext(csv_path)
    if ext != ".csv":
        raise Exception(f"{csv_path} is not a csv file.")
    
    data = np.loadtxt(csv_path, dtype=np.float32, delimiter=',')
    return data



def load_homography_data(homo_path):
    r""" Reading homography matrix from disk.
    
        Args:
            homo_path: string path of homography file.    
            
        Shape:
            - homography: numpy array with shape of `(3, 3)`.
    """
    if not os.path.exists(homo_path):
        raise Exception(f"{homo_path} doesn't exist.")
    
    data = np.loadtxt(homo_path, dtype=np.float32)
    return data



def transform_ref_segments(segments, homo_mat):
    r""" Apply homography transformation to line segments according reference image.
        example. H_1_2 mean transform reference image #1 to #2.
    
        Args:
            segments: numpy array of line segments, every segment is like `(x0, y0, x1, y1)`.
            homo_mat: homography matrix.

        Shape:
            segments: `(num_segments, 4)`.
            homo_mat: `(3, 3)`.
    """
    if segments.size == 0 or homo_mat.size == 0:
        raise Exception("Empty input array.")

    end_points0 = segments[:, :2].astype(np.float32)
    end_points1 = segments[:, 2:].astype(np.float32)

    w0 = np.ones((segments.shape[0], 1), dtype=np.float32)
    w1 = w0.copy()

    # transform shape to (3, num_segments)
    end_points0 = np.concatenate([end_points0, w0], axis=1).T
    end_points1 = np.concatenate([end_points1, w1], axis=1).T

    # apply homography matrix
    end_points0 = np.matmul(homo_mat, end_points0).T
    end_points1 = np.matmul(homo_mat, end_points1).T

    # normalize w
    end_points0 /= end_points0[:, -1:]
    end_points1 /= end_points1[:, -1:]

    return np.concatenate([end_points0[:, :2], end_points1[:, :2]], axis=1)