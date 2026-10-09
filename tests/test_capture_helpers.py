"""capture_calibration's pure helpers — the only parts testable without ROS."""

import numpy as np
import pytest

from capture_calibration import image_to_bgr


class FakeImage:
    def __init__(self, arr: np.ndarray, encoding: str, extra_stride: int = 0):
        self.height, self.width = arr.shape[:2]
        row = arr.reshape(self.height, -1)
        if extra_stride:
            row = np.hstack([row, np.zeros((self.height, extra_stride), np.uint8)])
        self.step = row.shape[1]
        self.data = row.tobytes()
        self.encoding = encoding


def test_bgr8_passthrough():
    img = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    out = image_to_bgr(FakeImage(img, "bgr8"))
    assert np.array_equal(out, img)


def test_rgb8_swaps_channels():
    img = np.arange(2 * 3 * 3, dtype=np.uint8).reshape(2, 3, 3)
    out = image_to_bgr(FakeImage(img, "rgb8"))
    assert np.array_equal(out, img[:, :, ::-1])


def test_row_padding_respected():
    img = np.arange(4 * 5 * 3, dtype=np.uint8).reshape(4, 5, 3)
    out = image_to_bgr(FakeImage(img, "bgr8", extra_stride=13))
    assert np.array_equal(out, img)


def test_mono8_becomes_bgr():
    img = np.arange(6 * 4, dtype=np.uint8).reshape(6, 4)
    out = image_to_bgr(FakeImage(img, "mono8"))
    assert out.shape == (6, 4, 3)
    assert np.array_equal(out[:, :, 0], img)


def test_unknown_encoding_raises():
    img = np.zeros((2, 2, 3), np.uint8)
    with pytest.raises(ValueError, match="16UC1"):
        image_to_bgr(FakeImage(img, "16UC1"))
