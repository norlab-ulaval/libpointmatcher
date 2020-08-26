import numpy as np


def list_modules():
    # TODO
    pass


def parse_translation(p_translation: str, p_cloud_dim: int):
    parsed_translation = np.identity(p_cloud_dim + 1)

    p_translation = p_translation.replace(',', ' ')

    translation_values = np.fromiter(p_translation.split(' '), np.float)

    for i, v in enumerate(translation_values):
        parsed_translation[i, p_cloud_dim] = v

    return parsed_translation


def parse_rotation(p_rotation: str, p_cloud_dim: int):
    parsed_rotation = np.identity(p_cloud_dim + 1)

    p_rotation = p_rotation.replace(',', ' ')
    p_rotation = p_rotation.replace(';', ' ')

    rotation_matrix = np.fromiter(p_rotation.split(' '), np.float)

    for i, v in enumerate(rotation_matrix):
        parsed_rotation[i // p_cloud_dim, i % p_cloud_dim] = v

    return parsed_rotation
