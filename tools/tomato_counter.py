#!/usr/bin/env python3
"""
tomato_counter.py - Kararli ve Kayip Iz Toleransli Domates Sayici Node
"""

import math
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Int32

RIPE, UNRIPE = 'ripe', 'unripe'

DEFAULTS = {
    'red_h_max': 12,
    'red_s_min': 100,
    'red_v_min': 80,
    
    'green_h_min': 18,
    'green_h_max': 48,
    'green_s_min': 70,
    'green_v_min': 80,
    
    'min_area': 40,
    'count_min_area': 130,   # Esik esnetildi ki kenarda kucuk gorunen son domates de sayılsın
    'max_aspect': 2.5,
    'line_frac': 0.42,
    'match_dist': 90,
    'max_missed': 8,
}


def image_to_bgr(msg):
    if msg.encoding not in ('rgb8', 'bgr8'):
        raise ValueError(f"Desteklenmeyen kodlama: {msg.encoding}")
    buf = np.frombuffer(msg.data, dtype=np.uint8)
    img = buf.reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'rgb8':
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return np.ascontiguousarray(img).copy()


def bgr_to_image(bgr, header):
    msg = Image()
    msg.header = header
    msg.height, msg.width = bgr.shape[:2]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    msg.data = bgr.tobytes()
    return msg


class Track:
    def __init__(self, tid, kind, cx, cy, area):
        self.id = tid
        self.kind = kind
        self.cx, self.cy = cx, cy
        self.max_area = area
        self.was_inside = False
        self.counted = False
        self.missed = 0


class CounterCore:
    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        if params:
            self.p.update(params)
        self.tracks = []
        self.next_id = 0
        self.count = {RIPE: 0, UNRIPE: 0}

    def _blobs(self, mask, kind):
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        out = []
        for c in cnts:
            area = cv2.contourArea(c)
            if area < self.p['min_area']:
                continue
            x, y, w, h = cv2.boundingRect(c)
            aspect = max(w / h, h / w)
            if aspect > self.p['max_aspect']:
                continue
            m = cv2.moments(c)
            if m['m00'] == 0:
                continue
            out.append({
                'kind': kind, 
                'cx': m['m10'] / m['m00'], 
                'cy': m['m01'] / m['m00'],
                'area': area, 
                'bbox': (x, y, w, h)
            })
        return out

    def detect(self, bgr):
        p = self.p
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        
        red = cv2.inRange(hsv, (0, p['red_s_min'], p['red_v_min']), (p['red_h_max'], 255, 255))
        red |= cv2.inRange(hsv, (180 - p['red_h_max'], p['red_s_min'], p['red_v_min']), (179, 255, 255))
        green = cv2.inRange(hsv, (p['green_h_min'], p['green_s_min'], p['green_v_min']), (p['green_h_max'], 255, 255))
        
        return self._blobs(red, RIPE) + self._blobs(green, UNRIPE)

    def update(self, dets, width):
        p = self.p
        mid = width / 2.0
        line_px = p['line_frac'] * width
        used, newly = set(), []

        for d in dets:
            best, best_dist = None, p['match_dist']
            for t in self.tracks:
                if t.kind != d['kind'] or t.id in used:
                    continue
                dist = math.hypot(t.cx - d['cx'], t.cy - d['cy'])
                if dist <= best_dist:
                    best, best_dist = t, dist
            
            if best is None:
                best = Track(self.next_id, d['kind'], d['cx'], d['cy'], d['area'])
                self.next_id += 1
                self.tracks.append(best)
            
            used.add(best.id)
            best.cx, best.cy = d['cx'], d['cy']
            best.missed = 0
            if d['area'] > best.max_area:
                best.max_area = d['area']
            
            d['track'] = best

            dx = abs(best.cx - mid)
            if dx < line_px:
                best.was_inside = True
            elif best.was_inside and not best.counted:
                if best.max_area >= p['count_min_area']:
                    best.counted = True
                    self.count[best.kind] += 1
                    newly.append(best.kind)

        for t in self.tracks:
            if t.id not in used:
                t.missed += 1
                # Eger domates merkeze girmisse ve cizgiyi gecmeden goruntuden cikip kayboluyorsa da say!
                if t.was_inside and not t.counted and t.missed > (p['max_missed'] // 2):
                    if t.max_area >= p['count_min_area']:
                        t.counted = True
                        self.count[t.kind] += 1
                        newly.append(t.kind)

        self.tracks = [t for t in self.tracks if t.missed <= p['max_missed']]
        return newly

    def annotate(self, bgr, dets):
        img = bgr.copy()
        h, w = img.shape[:2]
        mid, line_px = w // 2, int(self.p['line_frac'] * w)
        for x in (mid - line_px, mid + line_px):
            cv2.line(img, (x, 0), (x, h), (255, 255, 0), 1)
        for d in dets:
            x, y, bw, bh = d['bbox']
            t = d.get('track')
            color = (0, 0, 255) if d['kind'] == RIPE else (0, 255, 0)
            
            is_counted = t and t.counted
            cv2.rectangle(img, (x, y), (x + bw, y + bh), color, 3 if is_counted else 1)
            if t:
                cv2.putText(img, f"#{t.id}", (x, max(12, y - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
        
        txt = f"OLGUN: {self.count[RIPE]}   HAM: {self.count[UNRIPE]}"
        cv2.rectangle(img, (0, 0), (280, 26), (0, 0, 0), -1)
        cv2.putText(img, txt, (6, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        return img


class TomatoCounterNode(Node):
    def __init__(self):
        super().__init__('tomato_counter')
        for k, v in DEFAULTS.items():
            self.declare_parameter(k, v)
        self.core = CounterCore()

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.on_image, qos_profile_sensor_data)
        self.pub_ripe = self.create_publisher(Int32, '/ripe_count', 10)
        self.pub_unripe = self.create_publisher(Int32, '/unripe_count', 10)
        self.pub_debug = self.create_publisher(Image, '/camera/debug_image', 10)

    def on_image(self, msg):
        try:
            bgr = image_to_bgr(msg)
        except ValueError as e:
            return
            
        self.core.p.update({k: self.get_parameter(k).value for k in DEFAULTS})
        dets = self.core.detect(bgr)
        
        for kind in self.core.update(dets, bgr.shape[1]):
            name = 'OLGUN' if kind == RIPE else 'HAM'
            self.get_logger().info(f"{name} domates sayildi | OLGUN={self.core.count[RIPE]} HAM={self.core.count[UNRIPE]}")

        self.pub_ripe.publish(Int32(data=self.core.count[RIPE]))
        self.pub_unripe.publish(Int32(data=self.core.count[UNRIPE]))
        self.pub_debug.publish(bgr_to_image(self.core.annotate(bgr, dets), msg.header))


def main(args=None):
    rclpy.init(args=args)
    node = TomatoCounterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
