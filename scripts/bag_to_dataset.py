#!/usr/bin/env python3
"""Convert ROS1/ROS2 bags into dataset folders for `preprocess`, without ROS.

Reads bags with the pure-Python `rosbags` library. Each bag becomes one
sequence directory under OUT:

    OUT/<bag name>/image.png           first image (raw or compressed)
    OUT/<bag name>/camera_info.json    first camera info, if the bag has one
    OUT/<bag name>/points/<ns>.pcd     every point cloud, binary PCD, named by
                                       its header stamp in nanoseconds

All point fields are kept with their names and datatypes, so `preprocess`
sees the same data it would read from the bag.

    bag_to_dataset.py BAG [BAG ...] --out OUT
    bag_to_dataset.py DIR_OF_BAGS --out OUT
"""
import argparse
import json
import os
import sys
from pathlib import Path

import cv2
import numpy

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore

# sensor_msgs/PointField datatype -> (PCD type, size, numpy type)
POINT_FIELD_TYPES = {
  1: ('I', 1, 'i1'),
  2: ('U', 1, 'u1'),
  3: ('I', 2, 'i2'),
  4: ('U', 2, 'u2'),
  5: ('I', 4, 'i4'),
  6: ('U', 4, 'u4'),
  7: ('F', 4, 'f4'),
  8: ('F', 8, 'f8'),
}


def is_bag(path):
  if os.path.isfile(path):
    return path.endswith('.bag')
  return os.path.isfile(os.path.join(path, 'metadata.yaml'))


def find_bags(inputs):
  bags = []
  for path in inputs:
    path = os.path.normpath(path)
    if is_bag(path):
      bags.append(path)
    elif os.path.isdir(path):
      bags += sorted(os.path.join(path, name) for name in os.listdir(path) if is_bag(os.path.join(path, name)))
  return bags


def bag_name(path):
  name = os.path.basename(path)
  return name[:-4] if name.endswith('.bag') else name


def select_topic(connections, type_suffix, requested, label):
  topics = sorted({c.topic for c in connections if c.msgtype.endswith(type_suffix)})
  if requested:
    if requested not in {c.topic for c in connections}:
      sys.exit('error: topic %s not found' % requested)
    return requested
  if len(topics) > 1:
    sys.exit('error: multiple %s topics %s; choose one with --%s_topic' % (label, topics, label))
  return topics[0] if topics else None


def stamp_ns(header):
  return int(header.stamp.sec) * 1000000000 + int(header.stamp.nanosec)


def image_to_cv(msg, msgtype):
  if msgtype.endswith('CompressedImage'):
    return cv2.imdecode(numpy.frombuffer(msg.data, dtype=numpy.uint8), cv2.IMREAD_UNCHANGED)

  encoding = msg.encoding.lower()
  dtype = numpy.dtype(numpy.uint16 if '16' in encoding else numpy.uint8)
  if msg.is_bigendian:
    dtype = dtype.newbyteorder('>')
  channels = {'mono8': 1, 'mono16': 1, '8uc1': 1, '16uc1': 1, 'bgr8': 3, 'rgb8': 3, 'bgra8': 4, 'rgba8': 4}.get(encoding)
  if channels is None:
    sys.exit('error: unsupported image encoding %s' % msg.encoding)

  itemsize = dtype.itemsize
  rows = numpy.frombuffer(msg.data, dtype=numpy.uint8).reshape(msg.height, msg.step)
  image = rows[:, :msg.width * channels * itemsize].copy().view(dtype).reshape(msg.height, msg.width, channels).astype(dtype.newbyteorder('='))
  if encoding == 'rgb8':
    image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
  elif encoding == 'rgba8':
    image = cv2.cvtColor(image, cv2.COLOR_RGBA2BGRA)
  return image.squeeze()


def camera_info_to_json(msg):
  # ROS2 names the fields k/d, ROS1 K/D
  k = list(msg.k if hasattr(msg, 'k') else msg.K)
  d = list(msg.d if hasattr(msg, 'd') else msg.D)
  return {
    'distortion_model': msg.distortion_model,
    'width': int(msg.width),
    'height': int(msg.height),
    'intrinsics': [float(k[0]), float(k[4]), float(k[2]), float(k[5])],
    'distortion_coeffs': [float(x) for x in d],
  }


def write_pcd(path, msg, keep_fields):
  if msg.is_bigendian:
    sys.exit('error: big-endian point clouds are not supported')

  fields = [f for f in msg.fields if f.datatype in POINT_FIELD_TYPES and (not keep_fields or f.name in keep_fields)]
  num_points = msg.width * msg.height
  src_dtype = numpy.dtype({
    'names': [f.name for f in fields],
    'formats': [(POINT_FIELD_TYPES[f.datatype][2], (f.count,)) if f.count > 1 else POINT_FIELD_TYPES[f.datatype][2] for f in fields],
    'offsets': [f.offset for f in fields],
    'itemsize': msg.point_step,
  })
  src = numpy.frombuffer(numpy.asarray(msg.data, dtype=numpy.uint8).tobytes(), dtype=src_dtype, count=num_points)
  packed = numpy.empty(num_points, dtype=numpy.dtype([(name, src_dtype.fields[name][0]) for name in src_dtype.names]))
  for name in src_dtype.names:
    packed[name] = src[name]

  header = '\n'.join([
    '# .PCD v0.7 - Point Cloud Data file format',
    'VERSION 0.7',
    'FIELDS ' + ' '.join(f.name for f in fields),
    'SIZE ' + ' '.join(str(POINT_FIELD_TYPES[f.datatype][1]) for f in fields),
    'TYPE ' + ' '.join(POINT_FIELD_TYPES[f.datatype][0] for f in fields),
    'COUNT ' + ' '.join(str(max(f.count, 1)) for f in fields),
    'WIDTH %d' % msg.width,
    'HEIGHT %d' % msg.height,
    'VIEWPOINT 0 0 0 1 0 0 0',
    'POINTS %d' % num_points,
    'DATA binary',
  ]) + '\n'
  with open(path, 'wb') as f:
    f.write(header.encode('ascii'))
    f.write(packed.tobytes())


def convert(bag, out_dir, args, typestore):
  name = bag_name(bag)
  seq_dir = os.path.join(out_dir, name)
  points_dir = os.path.join(seq_dir, 'points')
  if os.path.exists(seq_dir) and not args.force:
    sys.exit('error: %s exists (use --force to overwrite)' % seq_dir)
  os.makedirs(points_dir, exist_ok=True)
  for old in os.listdir(points_dir):
    os.remove(os.path.join(points_dir, old))

  with AnyReader([Path(bag).resolve()], default_typestore=typestore) as reader:
    connections = reader.connections
    image_topic = select_topic(connections, 'Image', args.image_topic, 'image')
    info_topic = select_topic(connections, 'CameraInfo', args.camera_info_topic, 'camera_info')
    points_topic = select_topic(connections, 'PointCloud2', args.points_topic, 'points')
    print('%s: image=%s camera_info=%s points=%s' % (name, image_topic, info_topic, points_topic))
    if image_topic is None or points_topic is None:
      sys.exit('error: %s needs an image and a points topic' % bag)

    keep_fields = set(args.fields.split(',')) if args.fields else None
    have_image = False
    have_info = info_topic is None
    num_scans = 0
    stamps = set()
    for conn, _, rawdata in reader.messages(connections=[c for c in connections if c.topic in (image_topic, info_topic, points_topic)]):
      if conn.topic == image_topic and not have_image:
        msg = reader.deserialize(rawdata, conn.msgtype)
        image = image_to_cv(msg, conn.msgtype)
        cv2.imwrite(os.path.join(seq_dir, 'image.png'), image)
        have_image = True
      elif conn.topic == info_topic and not have_info:
        msg = reader.deserialize(rawdata, conn.msgtype)
        with open(os.path.join(seq_dir, 'camera_info.json'), 'w') as f:
          json.dump(camera_info_to_json(msg), f, indent=2)
        have_info = True
      elif conn.topic == points_topic:
        msg = reader.deserialize(rawdata, conn.msgtype)
        stamp = stamp_ns(msg.header)
        if stamp in stamps:
          print('warning: duplicate scan stamp %d; skipped' % stamp)
          continue
        stamps.add(stamp)
        write_pcd(os.path.join(points_dir, '%d.pcd' % stamp), msg, keep_fields)
        num_scans += 1

  print('%s: wrote image, %s and %d scans to %s' % (name, 'camera_info' if info_topic else 'no camera_info', num_scans, seq_dir))


def main():
  parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  parser.add_argument('inputs', nargs='+', help='ROS1 .bag files, ROS2 bag directories, or directories containing them')
  parser.add_argument('--out', required=True, help='output dataset directory')
  parser.add_argument('--image_topic', help='image topic (default: the only Image/CompressedImage topic)')
  parser.add_argument('--camera_info_topic', help='camera info topic (default: the only CameraInfo topic)')
  parser.add_argument('--points_topic', help='point cloud topic (default: the only PointCloud2 topic)')
  parser.add_argument('--fields', help='comma-separated point fields to keep (default: all)')
  parser.add_argument('--force', action='store_true', help='overwrite existing sequence directories')
  args = parser.parse_args()

  bags = find_bags(args.inputs)
  if not bags:
    sys.exit('error: no bags found in %s' % ' '.join(args.inputs))

  typestore = get_typestore(Stores.ROS2_HUMBLE)
  os.makedirs(args.out, exist_ok=True)
  for bag in bags:
    convert(bag, args.out, args, typestore)


if __name__ == '__main__':
  main()
