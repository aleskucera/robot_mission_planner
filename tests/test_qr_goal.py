"""Tests for the QR goal helpers (parser, debouncer, OpenCV decoding) - no ROS needed."""

import numpy as np
import pytest

from robot_mission_planner.qr_goal import Debouncer, decode_qr, parse_geo_uri

cv2 = pytest.importorskip("cv2")


# ── parser ──────────────────────────────────────────────────────────────────


@pytest.mark.parametrize(
    "text, expected",
    [
        ("geo:48.8016394,16.8011145", (48.8016394, 16.8011145)),
        ("GEO:48.8016394,16.8011145", (48.8016394, 16.8011145)),
        ("geo:50.1103476,14.4159857,230.5", (50.1103476, 14.4159857)),
        ("geo:50.1103476,14.4159857;u=10", (50.1103476, 14.4159857)),
        ("geo:50.1103476,14.4159857,230;crs=wgs84;u=35", (50.1103476, 14.4159857)),
        ("  geo: 50.1 , 14.4 \n", (50.1, 14.4)),
        ("50.1103476,14.4159857", (50.1103476, 14.4159857)),
        ("-33.9,151.2", (-33.9, 151.2)),
    ],
)
def test_parse_geo_uri_accepts(text, expected):
    assert parse_geo_uri(text) == pytest.approx(expected)


@pytest.mark.parametrize(
    "text",
    [
        "",
        None,
        "nonsense",
        "https://example.com/geo:50.1,14.4",
        "geo:50.1",
        "geo:91.0,14.4",  # latitude out of range
        "geo:50.1,181.0",  # longitude out of range
        "geo:0,0",  # osm2qr's cancel payload, never a goal
        "geo:50.1;14.4",
        "geo:abc,def",
    ],
)
def test_parse_geo_uri_rejects(text):
    assert parse_geo_uri(text) is None


# ── debouncer ───────────────────────────────────────────────────────────────


def test_debouncer_confirms_after_consecutive_frames():
    d = Debouncer(confirm_frames=2, republish_after_s=30.0)
    assert d.observe(["geo:1,2"], now=0.0) == []
    assert d.observe(["geo:1,2"], now=0.5) == ["geo:1,2"]
    assert d.observe(["geo:1,2"], now=1.0) == []  # already published
    assert d.observe([], now=1.5) == []  # streak broken
    assert d.observe(["geo:1,2"], now=2.0) == []  # needs two frames again ...
    assert d.observe(["geo:1,2"], now=2.5) == []  # ... but still within republish window
    assert d.observe(["geo:1,2"], now=31.0) == ["geo:1,2"]  # republished after 30 s


def test_debouncer_distinct_payloads_and_reset():
    d = Debouncer(confirm_frames=1, republish_after_s=30.0)
    assert d.observe(["a", "b", "a"], now=0.0) == ["a", "b"]
    assert d.observe(["a", "c"], now=1.0) == ["c"]
    d.reset()
    assert d.observe(["a"], now=2.0) == ["a"]


# ── OpenCV decoding ─────────────────────────────────────────────────────────


def _render_qr(text: str, size: int = 400, border: int = 40) -> np.ndarray:
    if not hasattr(cv2, "QRCodeEncoder"):
        pytest.skip("cv2.QRCodeEncoder not available")
    code = cv2.QRCodeEncoder.create().encode(text)
    big = cv2.resize(code, (size, size), interpolation=cv2.INTER_NEAREST)
    padded = cv2.copyMakeBorder(big, border, border, border, border, cv2.BORDER_CONSTANT, value=255)
    return cv2.cvtColor(padded, cv2.COLOR_GRAY2BGR)


def test_decode_rendered_qr_roundtrip():
    payload = "geo:50.1103476,14.4159857"
    img = _render_qr(payload)
    assert decode_qr(img) == [payload]
    assert parse_geo_uri(decode_qr(img)[0]) == pytest.approx((50.1103476, 14.4159857))


def test_decode_rotated_and_scaled_qr():
    payload = "geo:48.8016394,16.8011145"
    img = _render_qr(payload, size=300)
    rot90 = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
    assert decode_qr(rot90) == [payload]
    h, w = img.shape[:2]
    m = cv2.getRotationMatrix2D((w / 2, h / 2), 15, 0.8)
    tilted = cv2.warpAffine(img, m, (w, h), borderValue=(255, 255, 255))
    assert decode_qr(tilted) == [payload]
    small = cv2.resize(img, (120, 120), interpolation=cv2.INTER_AREA)
    assert decode_qr(small) == [payload]


def test_decode_qr_inside_a_scene():
    """A code occupying a corner of a larger, textured frame is still found."""
    payload = "geo:50.1,14.4"
    code = _render_qr(payload, size=200, border=20)
    rng = np.random.default_rng(0)
    scene = rng.integers(60, 200, size=(480, 640, 3), dtype=np.uint8)
    scene[200:440, 380:620] = code
    assert decode_qr(scene) == [payload]


def test_decode_blank_and_garbage_images():
    assert decode_qr(np.full((240, 320, 3), 255, dtype=np.uint8)) == []
    assert decode_qr(np.zeros((0, 0, 3), dtype=np.uint8)) == []
    assert decode_qr(None) == []
    rng = np.random.default_rng(1)
    assert decode_qr(rng.integers(0, 255, size=(200, 200, 3), dtype=np.uint8)) == []
