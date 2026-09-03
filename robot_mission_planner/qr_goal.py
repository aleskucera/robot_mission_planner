#!/usr/bin/env python3
"""
qr_goal: turn a Robotour goal QR code into a route_planner goal.

The competition hands the goal over as a QR code whose payload is a geo URI
(RFC 5870), e.g. ``geo:48.8016394,16.8011145``. This node reads the robot camera,
decodes QR codes with OpenCV, parses the payload and publishes the position as a
latched ``geographic_msgs/GeoPointStamped`` on ``/route_planner/goal`` - the
``route_planner`` node then plans from the robot's fix to it. The same payload can
be typed in on ``~/text`` (see ``qr_goal_send``) for the loading-zone goal, which
the team receives in the service area.

A payload is *confirmed* when it was decoded in ``confirm_frames`` consecutive
processed frames, and each distinct payload is published once (again only after
``republish_after_s``), so a code held in front of the camera does not spam goals.

Ported from vras-robotour/osm2qr ``qr2geo.py`` (pyzbar + nav2 FollowGPSWaypoints)
to OpenCV's ``QRCodeDetector`` and the Helhest route_planner interface.
"""

from __future__ import annotations

import re
import time

import numpy as np

try:  # OpenCV is a runtime dependency of the node; the parser works without it
    import cv2
except ImportError:  # pragma: no cover
    cv2 = None

# geo:lat,lon[,alt][;param=value...]  or a bare "lat,lon" - whitespace tolerant
_NUM = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
_GEO_RE = re.compile(
    rf"^\s*(?:geo:)?\s*({_NUM})\s*,\s*({_NUM})\s*(?:,\s*{_NUM}\s*)?(?:;.*)?$",
    re.IGNORECASE | re.DOTALL,
)


def parse_geo_uri(text: str | None) -> tuple[float, float] | None:
    """
    ``(lat, lon)`` from a geo URI or a bare ``lat,lon`` string, else ``None``.

    Accepts an optional altitude and ``;`` parameters (``geo:50.1,14.4,230;u=10``),
    any case for the scheme and surrounding whitespace. Rejects values outside
    +-90 / +-180 and the ``0,0`` payload (osm2qr used it as "cancel"; it is never
    a Robotour goal). Never raises.
    """
    if not text:
        return None
    m = _GEO_RE.match(str(text))
    if not m:
        return None
    try:
        lat, lon = float(m.group(1)), float(m.group(2))
    except ValueError:
        return None
    if not (abs(lat) <= 90.0 and abs(lon) <= 180.0):
        return None
    if lat == 0.0 and lon == 0.0:
        return None
    return lat, lon


def decode_qr(image) -> list[str]:
    """Non-empty QR payloads found in a BGR/grey image (OpenCV QRCodeDetector)."""
    return [text for text, _ in decode_qr_with_points(image)]


def decode_qr_with_points(image) -> list[tuple[str, np.ndarray | None]]:
    """``[(payload, 4x2 corner points or None), ...]``; ``[]`` on any failure."""
    if cv2 is None or image is None or getattr(image, "size", 0) == 0:
        return []
    detector = cv2.QRCodeDetector()
    out: list[tuple[str, np.ndarray | None]] = []
    try:
        if hasattr(detector, "detectAndDecodeMulti"):
            ok, texts, points, _ = detector.detectAndDecodeMulti(image)
            if ok:
                for i, text in enumerate(texts):
                    if text:
                        pts = points[i] if points is not None and i < len(points) else None
                        out.append((text, pts))
        if not out:
            text, points, _ = detector.detectAndDecode(image)
            if text:
                out.append((text, points[0] if points is not None and len(points) else None))
    except cv2.error:
        return []
    except Exception:  # noqa: BLE001 - a bad frame must never kill the node
        return []
    return out


class Debouncer:
    """
    Confirms payloads seen in ``confirm_frames`` consecutive observations and
    reports each distinct payload once, again only after ``republish_after_s``.

    ``observe(payloads, now)`` is called once per processed frame with every
    payload decoded from it (possibly none) and returns the payloads to act on.
    """

    def __init__(self, confirm_frames: int = 2, republish_after_s: float = 30.0) -> None:
        self.confirm_frames = max(1, int(confirm_frames))
        self.republish_after_s = float(republish_after_s)
        self._streak: dict[str, int] = {}
        self._published: dict[str, float] = {}

    def observe(self, payloads, now: float | None = None) -> list[str]:
        now = time.monotonic() if now is None else float(now)
        seen = set(payloads)
        self._streak = {p: self._streak.get(p, 0) + 1 for p in seen}
        confirmed = []
        for p in payloads:
            if p in confirmed or self._streak[p] < self.confirm_frames:
                continue
            last = self._published.get(p)
            if last is not None and (now - last) < self.republish_after_s:
                continue
            self._published[p] = now
            confirmed.append(p)
        return confirmed

    def reset(self) -> None:
        self._streak.clear()
        self._published.clear()


# ---------------------------------------------------------------------- ROS node


def _decode_image_msg(msg, transport: str):
    """sensor_msgs CompressedImage / Image -> BGR (or grey) numpy image, or None."""
    if transport == "compressed":
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
        return cv2.imdecode(buf, cv2.IMREAD_COLOR)
    enc = msg.encoding.lower()
    h, w = int(msg.height), int(msg.width)
    data = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    if enc in ("bgr8", "rgb8"):
        img = data.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        return img if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc in ("mono8", "8uc1"):
        return data.reshape(h, msg.step)[:, :w].copy()
    if enc in ("bgra8", "rgba8"):
        img = data.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    if enc.startswith("bayer"):
        raw = data.reshape(h, msg.step)[:, :w].copy()
        code = {
            "bayer_rggb8": cv2.COLOR_BayerBG2BGR,
            "bayer_bggr8": cv2.COLOR_BayerRG2BGR,
            "bayer_gbrg8": cv2.COLOR_BayerGR2BGR,
            "bayer_grbg8": cv2.COLOR_BayerGB2BGR,
        }.get(enc)
        return cv2.cvtColor(raw, code) if code else raw
    try:  # anything else: let cv_bridge deal with it if it is around
        from cv_bridge import CvBridge

        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    except Exception:  # noqa: BLE001
        return None


def main(args=None):
    import rclpy
    from geographic_msgs.msg import GeoPointStamped
    from rclpy.node import Node
    from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage, Image
    from std_msgs.msg import String
    from std_srvs.srv import SetBool

    class QrGoal(Node):
        def __init__(self):
            super().__init__("qr_goal")
            p = self.declare_parameter
            self.image_topic = p("image_topic", "/camera/image_color/compressed").value
            self.transport = p("image_transport", "compressed").value  # compressed | raw
            self.process_rate = float(p("process_rate", 4.0).value)  # Hz, frames above are dropped
            confirm_frames = int(p("confirm_frames", 2).value)
            republish_after = float(p("republish_after_s", 30.0).value)
            self.goal_topic = p("goal_topic", "/route_planner/goal").value
            self.goal_frame_id = p("goal_frame_id", "wgs84").value
            text_topic = p("text_topic", "~/text").value
            detections_topic = p("detections_topic", "~/detections").value
            self.publish_annotated = bool(p("publish_annotated", False).value)
            self.enabled = bool(p("enabled", True).value)

            if self.transport not in ("compressed", "raw"):
                self.get_logger().error(
                    f"image_transport must be compressed|raw, got '{self.transport}'; using compressed"
                )
                self.transport = "compressed"
            if cv2 is None:
                self.get_logger().error("OpenCV (python3-opencv) is missing: camera decoding disabled")

            latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
            self.pub_goal = self.create_publisher(GeoPointStamped, self.goal_topic, latched)
            self.pub_detections = self.create_publisher(String, detections_topic, 10)
            self.pub_annotated = (
                self.create_publisher(Image, "~/image_annotated", 1) if self.publish_annotated else None
            )
            # latched: qr_goal_send publishes once and exits before a volatile match would happen
            self.create_subscription(String, text_topic, self._text_cb, latched)
            self.create_service(SetBool, "~/enable", self._enable_cb)
            msg_type = CompressedImage if self.transport == "compressed" else Image
            self.create_subscription(msg_type, self.image_topic, self._image_cb, qos_profile_sensor_data)

            self.debouncer = Debouncer(confirm_frames, republish_after)
            self._last_processed = 0.0
            self._warned_payloads: set[str] = set()
            self._frames = 0
            self.create_timer(30.0, self._heartbeat)
            self.get_logger().info(
                f"qr_goal ready: {self.image_topic} ({self.transport}) at <= {self.process_rate:g} Hz, "
                f"confirm {confirm_frames} frames, goals -> {self.goal_topic}, text on {text_topic}"
            )

        # -------------------------------------------------------------- inputs
        def _image_cb(self, msg):
            self._frames += 1
            if not self.enabled or cv2 is None:
                return
            now = time.monotonic()
            if self.process_rate > 0 and (now - self._last_processed) < 1.0 / self.process_rate:
                return
            self._last_processed = now
            try:
                image = _decode_image_msg(msg, self.transport)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warn(f"cannot decode image: {e}", throttle_duration_sec=10.0)
                return
            if image is None:
                self.get_logger().warn("cannot decode image (unknown encoding?)", throttle_duration_sec=10.0)
                return
            found = decode_qr_with_points(image)
            payloads = [t for t, _ in found]
            for t in payloads:
                self.pub_detections.publish(String(data=t))
            for payload in self.debouncer.observe(payloads, now):
                self._handle_payload(payload, "camera")
            if self.pub_annotated is not None:
                self._publish_annotated(image, found, msg.header)

        def _text_cb(self, msg):
            self._handle_payload(msg.data, "text")

        def _enable_cb(self, req, res):
            self.enabled = bool(req.data)
            if not self.enabled:
                self.debouncer.reset()
            res.success = True
            res.message = "qr_goal detection " + ("enabled" if self.enabled else "disabled")
            self.get_logger().info(res.message)
            return res

        # -------------------------------------------------------------- output
        def _handle_payload(self, payload: str, source: str):
            latlon = parse_geo_uri(payload)
            if latlon is None:
                if payload not in self._warned_payloads:
                    self._warned_payloads.add(payload)
                    self.get_logger().warn(f"ignoring {source} payload without a geo position: {payload!r}")
                return
            lat, lon = latlon
            msg = GeoPointStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.goal_frame_id
            msg.position.latitude = lat
            msg.position.longitude = lon
            self.pub_goal.publish(msg)
            self.get_logger().info(f"GOAL from {source}: {payload!r} -> {lat:.7f}, {lon:.7f} on {self.goal_topic}")

        def _publish_annotated(self, image, found, header):
            img = image if image.ndim == 3 else cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            img = img.copy()
            for text, pts in found:
                if pts is not None:
                    poly = np.asarray(pts, dtype=np.int32).reshape(-1, 1, 2)
                    cv2.polylines(img, [poly], True, (0, 255, 0), 3)
                    x, y = poly[0, 0]
                    cv2.putText(img, text[:40], (int(x), max(20, int(y) - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            out = Image()
            out.header = header
            out.height, out.width = img.shape[:2]
            out.encoding = "bgr8"
            out.step = out.width * 3
            out.data = img.tobytes()
            self.pub_annotated.publish(out)

        def _heartbeat(self):
            if self._frames == 0:
                self.get_logger().warn(f"no images received on {self.image_topic} yet", throttle_duration_sec=60.0)

    rclpy.init(args=args)
    node = QrGoal()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
