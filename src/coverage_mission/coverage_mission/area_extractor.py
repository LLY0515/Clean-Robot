#!/usr/bin/env python3
"""从 Nav2 2D 栅格地图提取可清扫区域，生成 coverage_area.yaml。

处理链：
  读 my_map.yaml/pgm -> 自由区二值化 -> 形态学去噪 -> 按机器人半径腐蚀
  -> 提取外轮廓与内部孔洞（柱子） -> 简化多边形 -> 写入 YAML

用法：
  ros2 run coverage_mission extract_coverage_area \
      --map src/sam_bot_description/map/my_map.yaml \
      --output src/coverage_mission/config/coverage_area.yaml \
      --margin 0.55
"""

import argparse
import math
import os

import cv2
import numpy as np
import yaml


def load_map(map_yaml):
    with open(map_yaml, 'r') as f:
        meta = yaml.safe_load(f)

    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(os.path.abspath(map_yaml)),
                                  image_path)
    img = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
    if img is None:
        raise RuntimeError('无法读取地图图片：%s' % image_path)
    if img.ndim == 3:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    negate = int(meta.get('negate', 0))
    if negate:
        # Nav2 语义：negate=1 时像素 0 表示自由
        img = 255 - img

    # map_saver 的约定：0=占据，254=自由，205=未知。
    # 只把 >=250 当作自由，未知区域不参与清扫规划。
    free = img >= 250
    occupied = img <= 10
    return meta, img, free, occupied


def signed_area(points):
    area = 0.0
    for i in range(len(points)):
        x1, y1 = points[i]
        x2, y2 = points[(i + 1) % len(points)]
        area += x1 * y2 - x2 * y1
    return area * 0.5


def dedupe(points, eps=1e-6):
    out = []
    for p in points:
        if not out or math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) > eps:
            out.append(p)
    if len(out) > 1 and math.hypot(out[0][0] - out[-1][0],
                                   out[0][1] - out[-1][1]) <= eps:
        out.pop()
    return out


def contour_to_world(contour, meta, img_height, epsilon_px):
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])

    approx = cv2.approxPolyDP(contour, epsilon_px, True)
    pts = []
    for p in approx:
        px = float(p[0][0])
        py = float(p[0][1])
        x = ox + (px + 0.5) * res
        # 图像行号向下增长，地图 y 向上增长
        y = oy + (img_height - py - 0.5) * res
        pts.append([round(float(x), 3), round(float(y), 3)])

    pts = dedupe(pts)
    if len(pts) < 3:
        return []
    # 统一环方向：外环 CCW（signed_area > 0），孔洞 CW
    return pts


def point_in_polygon(x, y, polygon):
    inside = False
    n = len(polygon)
    for i in range(n):
        x1, y1 = polygon[i]
        x2, y2 = polygon[(i + 1) % n]
        if (y1 > y) != (y2 > y):
            x_cross = (x2 - x1) * (y - y1) / (y2 - y1) + x1
            if x < x_cross:
                inside = not inside
    return inside


def zone_contains(zone, x, y):
    if not point_in_polygon(x, y, zone['outer']):
        return False
    for hole in zone['holes']:
        if point_in_polygon(x, y, hole):
            return False
    return True


def pick_start_pose(zones, mask, meta, biggest_index):
    """默认起点：最大分区内距离障碍最远的自由点。"""
    res = float(meta['resolution'])
    ox, oy = float(meta['origin'][0]), float(meta['origin'][1])
    h, w = mask.shape

    # 只在最大分区内找距离变换峰值
    zone_mask = np.zeros((h, w), dtype=np.uint8)
    zone = zones[biggest_index]
    poly = np.array(
        [[int(round((x - ox) / res - 0.5)),
          int(round(h - (y - oy) / res - 0.5))] for x, y in zone['outer']],
        dtype=np.int32)
    cv2.fillPoly(zone_mask, [poly], 255)
    for hole in zone['holes']:
        hpoly = np.array(
            [[int(round((x - ox) / res - 0.5)),
              int(round(h - (y - oy) / res - 0.5))] for x, y in hole],
            dtype=np.int32)
        cv2.fillPoly(zone_mask, [hpoly], 0)

    dist = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    dist[zone_mask == 0] = 0.0
    py, px = np.unravel_index(int(np.argmax(dist)), dist.shape)

    x = ox + (px + 0.5) * res
    y = oy + (h - py - 0.5) * res
    return round(float(x), 3), round(float(y), 3)


def extract(map_yaml, margin, wall_close, min_zone_area, min_hole_area,
            epsilon, operation_width, start_xy=None):
    meta, img, free, occupied = load_map(map_yaml)
    res = float(meta['resolution'])
    img_h, img_w = img.shape

    # (1) 先把障碍物（墙）膨胀 wall_close 米，封住门缝/墙缝，
    #     否则房间内部会和地图边缘的自由区连成一片，区域会覆盖整张图。
    wall_px = int(round(wall_close / res))
    occupied_u8 = (occupied.astype(np.uint8) * 255)
    if wall_px > 0:
        k = 2 * wall_px + 1
        sealed_occ = cv2.dilate(
            occupied_u8,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    else:
        sealed_occ = occupied_u8
    seeded_free = ((free.astype(np.uint8) * 255)
                   & cv2.bitwise_not(sealed_occ))

    # (2) 只保留"不接触图像边界"的连通区：接触边界的通常是墙外的那圈自由区。
    #     多个互不连通的房间会分别成为独立分区。
    num, labels, stats, _ = cv2.connectedComponentsWithStats(seeded_free, 8)
    mask = np.zeros((img_h, img_w), dtype=np.uint8)
    selected = 0
    for i in range(1, num):
        area = stats[i, cv2.CC_STAT_AREA] * res * res
        if area < max(1.0, min_zone_area):
            continue
        x, y, w, h = (stats[i, cv2.CC_STAT_LEFT], stats[i, cv2.CC_STAT_TOP],
                      stats[i, cv2.CC_STAT_WIDTH], stats[i, cv2.CC_STAT_HEIGHT])
        touches_border = (x <= 0 or y <= 0 or
                          x + w >= img_w or y + h >= img_h)
        if touches_border:
            continue
        mask[labels == i] = 255
        selected += 1

    if selected == 0:
        # 地图本身被裁剪到墙边时，退回"包含地图中心"的连通区
        cx, cy = img_w // 2, img_h // 2
        label = labels[cy, cx]
        if label == 0:
            raise RuntimeError('地图中没有可用的内部自由区')
        mask[labels == label] = 255

    # (3) 闭运算补小裂缝，开运算去小噪点
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)))
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN,
        cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))

    # (4) 腐蚀出机器人安全边距。封缝时已经向内收了 wall_close，这里补差额。
    radius_px = int(round(max(0.0, margin - wall_close) / res))
    if radius_px > 0:
        k = 2 * radius_px + 1
        mask = cv2.erode(
            mask, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))

    contours, hierarchy = cv2.findContours(
        mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if not contours or hierarchy is None:
        raise RuntimeError('地图中没有提取到可清扫区域')

    hierarchy = hierarchy[0]
    epsilon_px = max(1.0, epsilon / res)

    zones = []
    for i, contour in enumerate(contours):
        if hierarchy[i][3] != -1:      # 只处理顶层外轮廓
            continue
        area = cv2.contourArea(contour) * res * res
        if area < min_zone_area:
            continue

        outer = contour_to_world(contour, meta, img_h, epsilon_px)
        if len(outer) < 3:
            continue
        if signed_area(outer) < 0:
            outer.reverse()

        holes = []
        child = hierarchy[i][2]
        while child != -1:
            hole_area = cv2.contourArea(contours[child]) * res * res
            if hole_area >= min_hole_area:
                hole = contour_to_world(contours[child], meta, img_h,
                                        epsilon_px)
                if len(hole) >= 3:
                    if signed_area(hole) > 0:
                        hole.reverse()
                    holes.append(hole)
            child = hierarchy[child][0]

        zones.append({
            'name': 'zone_%d' % (len(zones) + 1),
            'area': round(float(area), 2),
            'outer': outer,
            'holes': holes,
        })

    if not zones:
        raise RuntimeError('区域面积都小于 min_zone_area=%.2f m²' %
                           min_zone_area)

    # 按面积从大到小排序，最大分区作为默认起点所在分区
    zones.sort(key=lambda z: z['area'], reverse=True)
    biggest = 0

    if start_xy is not None:
        inside = any(zone_contains(z, start_xy[0], start_xy[1])
                     for z in zones)
        if not inside:
            print('[警告] 指定起点 (%.2f, %.2f) 不在任何清扫区域内，'
                  '改用自动选点' % (start_xy[0], start_xy[1]))
            start_xy = None

    if start_xy is None:
        sx, sy = pick_start_pose(zones, mask, meta, biggest)
    else:
        sx, sy = float(start_xy[0]), float(start_xy[1])

    data = {
        'frame_id': 'map',
        'source_map': os.path.abspath(map_yaml),
        'resolution': res,
        'operation_width': float(operation_width),
        'start_pose': [round(sx, 3), round(sy, 3), 0.0],
        'zones': zones,
    }
    return data


def main(args=None):
    parser = argparse.ArgumentParser(
        description='从 Nav2 栅格地图提取全覆盖清扫区域')
    parser.add_argument('--map', required=True,
                        help='输入地图 yaml（如 my_map.yaml）')
    parser.add_argument('--output', required=True,
                        help='输出 coverage_area.yaml')
    parser.add_argument('--margin', type=float, default=0.55,
                        help='自由区腐蚀边距（m），建议机器人半宽+0.05')
    parser.add_argument('--wall-close', type=float, default=0.25,
                        help='墙体膨胀量（m），用于封住门缝、分离房间内外')
    parser.add_argument('--min-zone-area', type=float, default=2.0,
                        help='最小分区面积（m²）')
    parser.add_argument('--min-hole-area', type=float, default=0.4,
                        help='最小孔洞面积（m²），过滤噪点')
    parser.add_argument('--epsilon', type=float, default=0.08,
                        help='多边形简化容差（m）')
    parser.add_argument('--operation-width', type=float, default=0.8,
                        help='洗地盘宽度（m），写入配置供参考')
    parser.add_argument('--start', type=float, nargs=2, default=None,
                        metavar=('X', 'Y'),
                        help='指定清扫起点（map 帧）')
    args, _ = parser.parse_known_args(args)

    data = extract(args.map, args.margin, args.wall_close, args.min_zone_area,
                   args.min_hole_area, args.epsilon, args.operation_width,
                   args.start)

    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w') as f:
        yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)

    total = sum(z['area'] for z in data['zones'])
    print('已写入 %s' % args.output)
    print('  分区: %d，总面积: %.1f m²，起点: %s' %
          (len(data['zones']), total, data['start_pose']))
    for z in data['zones']:
        print('  - %s: %.1f m²，外环 %d 点，孔洞 %d 个' %
              (z['name'], z['area'], len(z['outer']), len(z['holes'])))
    if data['operation_width'] > 0:
        print('  预计条带数: %d（operation_width=%.2f m）' %
              (int(math.ceil(total / data['operation_width'])),
               data['operation_width']))


if __name__ == '__main__':
    main()
