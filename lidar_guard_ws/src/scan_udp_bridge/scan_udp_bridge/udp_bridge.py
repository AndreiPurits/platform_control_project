#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import argparse, socket, json, math

def to_laserscan_from_pts(msg, angle_min, angle_max, angle_inc, rmin, rmax):
    pts = msg.get('pts') or []
    n = int(round((angle_max - angle_min) / angle_inc)) + 1
    ranges = [None] * n
    for p in pts:
        if not isinstance(p, (list, tuple)) or len(p) < 2:
            continue
        x, y = float(p[0]), float(p[1])
        r = math.hypot(x, y)
        if not (rmin <= r <= rmax):
            continue
        a = math.atan2(y, x)
        while a < angle_min:
            a += 2 * math.pi
        while a > angle_max:
            a -= 2 * math.pi
        if not (angle_min <= a <= angle_max):
            continue
        idx = int(round((a - angle_min) / angle_inc))
        if 0 <= idx < n and (ranges[idx] is None or r < ranges[idx]):
            ranges[idx] = r
    return {
        "angle_min": angle_min,
        "angle_increment": angle_inc,
        "range_min": rmin,
        "range_max": rmax,
        "ranges": ranges
    }

def to_pts_from_laserscan(msg):
    angle = float(msg.get("angle_min", 0.0))
    inc = float(msg.get("angle_increment", 0.0))
    rmin = float(msg.get("range_min", 0.0))
    rmax = float(msg.get("range_max", 20.0))
    ranges = msg.get("ranges", [])
    pts = []
    for r in ranges:
        if r is None:
            angle += inc
            continue
        rr = float(r)
        if rr <= 0.0 or rr < rmin or rr > rmax:
            angle += inc
            continue
        pts.append([rr * math.cos(angle), rr * math.sin(angle)])
        angle += inc
    return {"pts": pts}

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in_host', default='0.0.0.0')
    ap.add_argument('--in_port', type=int, default=9999)
    ap.add_argument('--out_host', default='127.0.0.1')
    ap.add_argument('--out_port', type=int, default=10000)
    ap.add_argument('--to', choices=['passthrough', 'laserscan', 'pts'], default='passthrough')
    ap.add_argument('--angle_min', type=float, default=-math.pi)
    ap.add_argument('--angle_max', type=float, default= math.pi)
    ap.add_argument('--angle_increment_deg', type=float, default=1.0)
    ap.add_argument('--range_min', type=float, default=0.05)
    ap.add_argument('--range_max', type=float, default=20.0)
    args = ap.parse_args()

    angle_inc = math.radians(args.angle_increment_deg)

    rsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rsock.bind((args.in_host, args.in_port))
    wsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    while True:
        data, _ = rsock.recvfrom(1<<20)
        try:
            msg = json.loads(data.decode('utf-8'))
        except Exception:
            continue

        if args.to == 'passthrough':
            wsock.sendto(data, (args.out_host, args.out_port))
            continue

        if args.to == 'laserscan':
            if 'pts' in msg:
                out = to_laserscan_from_pts(
                    msg, args.angle_min, args.angle_max, angle_inc, args.range_min, args.range_max
                )
            elif all(k in msg for k in ('angle_min','angle_increment','ranges')):
                out = msg
            else:
                continue
        elif args.to == 'pts':
            if all(k in msg for k in ('angle_min','angle_increment','ranges')):
                out = to_pts_from_laserscan(msg)
            elif 'pts' in msg:
                out = msg
            else:
                continue
        else:
            continue

        wsock.sendto(json.dumps(out, separators=(',', ':')).encode('utf-8'), (args.out_host, args.out_port))

if __name__ == '__main__':
    main()