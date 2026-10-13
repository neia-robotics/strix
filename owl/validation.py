"""Shared validation for file, web and NetworkTables settings."""
import math
import re
import cv2


def number(name, value, low, high, *, integer=False, positive=False):
    if isinstance(value, bool):
        raise ValueError(f"{name} must be numeric")
    v = float(value)
    if not math.isfinite(v) or not low <= v <= high or (positive and v <= 0):
        raise ValueError(f"{name} must be finite and between {low} and {high}")
    if integer and v != int(v):
        raise ValueError(f"{name} must be an integer")
    return int(v) if integer else v


def setting(name, value):
    if name == 'resolution':
        if not isinstance(value, str) or not re.fullmatch(r'\d+x\d+', value):
            raise ValueError('resolution must be WIDTHxHEIGHT')
        w, h = (number(name, x, 16, 8192, integer=True) for x in value.split('x'))
        return f'{w}x{h}'
    if name == 'board_dict':
        if not isinstance(value, str):
            raise ValueError('board_dict must be a dictionary name')
        value = value.strip().upper()
        if not value.startswith('DICT_') or not hasattr(cv2.aruco, value):
            raise ValueError('unknown ArUco dictionary')
        cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, value))
        return value
    if name == 'board_legacy':
        if not isinstance(value, bool):
            raise ValueError('board_legacy must be true or false')
        return value
    if name in ('exposure', 'gain'):
        v = number(name, -1 if value is None else value, -1, 1e6)
        if -1 < v < 0:
            raise ValueError(f'{name} must be -1 (automatic) or nonnegative')
        return v
    bounds = {
        'quad_decimate': (.5, 8), 'min_cluster_pixels': (1, 100000),
        'min_decision_margin': (0, 1000), 'max_single_tag_ambiguity': (0, 1),
        'max_tag_distance_m': (.01, 100), 'max_reprojection_error_px': (.01, 100),
        'stream_max_fps': (0, 240), 'board_squares_x': (2, 30), 'board_squares_y': (2, 30),
        'board_square_length_m': (.001, 1), 'board_marker_length_m': (.001, 1),
        'std_dev_factor': (1e-6, 1e6),
    }
    if name not in bounds:
        raise ValueError(f'unknown setting: {name}')
    return number(name, value, *bounds[name], integer=name in (
        'min_cluster_pixels', 'board_squares_x', 'board_squares_y'))


def validate_settings(values):
    result = {k: setting(k, v) for k, v in values.items()}
    if 'board_square_length_m' in result:
        if result['board_marker_length_m'] >= result['board_square_length_m']:
            raise ValueError('board marker must be smaller than its square')
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, result['board_dict']))
        if result['board_squares_x'] * result['board_squares_y'] // 2 > len(dictionary.bytesList):
            raise ValueError('board needs more markers than the dictionary contains')
    return result
