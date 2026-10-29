#!/usr/bin/env python3
# Camera-LiDAR image matching for initial_guess_auto with DaD keypoints and the
# RoMa dense matcher (both MIT licensed; the weights are RoMa outdoor, DINOv2
# ViT-L/14 (Apache-2.0) and DaD v0.1.0).
#
# DaD detects keypoints in the camera image and the LiDAR intensity image, RoMa
# estimates a dense warp from the camera image to the LiDAR image, and keypoints
# are matched where the warped camera keypoint lands on a LiDAR keypoint (mutual
# nearest neighbours). The output is the <bag>_matches.json that
# initial_guess_auto reads, in the format of upstream find_matches_superglue.py.
import os
import json
import argparse
import tempfile

import cv2
import numpy
import torch
import matplotlib
import torch.nn.functional as F
from PIL import Image


def angle_to_rot(angle, image_shape):
  """cv2 rotation code for a clockwise rotation, and the map from rotated pixel
  coordinates back to the original image."""
  h, w = image_shape[:2]
  if angle == 90:
    return cv2.ROTATE_90_CLOCKWISE, lambda x: numpy.stack([x[:, 1], h - 1 - x[:, 0]], axis=1)
  if angle == 180:
    return cv2.ROTATE_180, lambda x: numpy.stack([w - 1 - x[:, 0], h - 1 - x[:, 1]], axis=1)
  if angle == 270:
    return cv2.ROTATE_90_COUNTERCLOCKWISE, lambda x: numpy.stack([w - 1 - x[:, 1], x[:, 0]], axis=1)
  raise SystemExit('error: unsupported rotation angle %d (use 90, 180 or 270)' % angle)


def load_models(opt, device):
  # Imported here so that --help works without the models installed
  import dad
  from romatch import roma_outdoor

  # Weights come from the torch hub cache ($TORCH_HOME), which the Docker image
  # fills at build time; elsewhere they are downloaded on first use.
  amp_dtype = torch.float16 if device.type == 'cuda' else torch.float32
  matcher = roma_outdoor(
    device=device,
    coarse_res=opt.coarse_res,
    upsample_res=tuple(opt.upsample_res),
    amp_dtype=amp_dtype,
    symmetric=False,
    use_custom_corr=False,
  )
  matcher.eval()

  detector = dad.load_DaD().to(device).eval()
  if device.type != 'cuda':
    # DaD always runs its decoder under float16 autocast, which on the CPU
    # takes a slow single-threaded path (minutes per image instead of seconds)
    for module in detector.modules():
      if hasattr(module, 'amp'):
        module.amp = False
  return matcher, detector


def detect(detector, image_path, num_keypoints, device):
  batch = detector.load_image(image_path, device=device)
  return detector.detect(batch, num_keypoints=num_keypoints)['keypoints'][0]


def match_images(matcher, detector, camera_image, lidar_image, opt, device):
  """Returns DaD keypoints of both images in pixels and, for each camera
  keypoint, the index of the matched LiDAR keypoint (-1 if none) and the RoMa
  certainty of the match."""
  with tempfile.TemporaryDirectory() as tmpdir:
    # RoMa and DaD load RGB images from files, so save the (rotated) inputs
    camera_path = os.path.join(tmpdir, 'camera.png')
    lidar_path = os.path.join(tmpdir, 'lidar.png')
    cv2.imwrite(camera_path, camera_image)
    cv2.imwrite(lidar_path, lidar_image)

    warp, certainty = matcher.match(camera_path, lidar_path, device=device)
    kpts0 = detect(detector, camera_path, opt.num_keypoints, device)
    kpts1 = detect(detector, lidar_path, opt.num_keypoints, device)

  warp, certainty = warp[0], certainty[0]
  inds0, inds1 = matcher.match_keypoints(kpts0, kpts1, warp, certainty, return_tuple=True, return_inds=True, max_dist=opt.max_dist, cert_th=opt.certainty_thresh)

  # RoMa certainty at the matched camera keypoints
  match_certainty = F.grid_sample(certainty[None, None], kpts0[inds0][None, None], align_corners=False, mode='bilinear')[0, 0, 0]

  matches = numpy.full(len(kpts0), -1, dtype=numpy.int64)
  confidence = numpy.zeros(len(kpts0))
  matches[inds0.cpu().numpy()] = inds1.cpu().numpy()
  confidence[inds0.cpu().numpy()] = match_certainty.cpu().numpy()

  # Normalized coordinates -> pixel indices (pixel centres at integers, as initial_guess_auto expects)
  def to_pixels(kpts, image):
    h, w = image.shape[:2]
    return (detector.to_pixel_coords(kpts, h, w) - detector.topleft).cpu().numpy()

  return to_pixels(kpts0, camera_image), to_pixels(kpts1, lidar_image), matches, confidence


def draw_matches(camera_image, lidar_image, kpts0, kpts1, matches, confidence):
  camera_canvas = cv2.cvtColor(camera_image, cv2.COLOR_GRAY2BGR)
  lidar_canvas = cv2.cvtColor(lidar_image, cv2.COLOR_GRAY2BGR)
  lidar_canvas = cv2.resize(lidar_canvas, (camera_image.shape[1], camera_image.shape[0]))

  sx = camera_image.shape[1] / lidar_image.shape[1]
  sy = camera_image.shape[0] / lidar_image.shape[0]
  kpts1 = numpy.stack([kpts1[:, 0] * sx + camera_image.shape[1], kpts1[:, 1] * sy], axis=1)

  canvas = numpy.concatenate([camera_canvas, lidar_canvas], axis=1)
  for kp in kpts0:
    cv2.circle(canvas, (int(kp[0]), int(kp[1])), 3, (255, 255, 255))
  for kp in kpts1:
    cv2.circle(canvas, (int(kp[0]), int(kp[1])), 3, (255, 255, 255))

  cmap = matplotlib.colormaps['turbo']
  for i, match in enumerate(matches):
    if match < 0:
      continue
    kp0 = kpts0[i]
    kp1 = kpts1[match]
    color = tuple((numpy.array(cmap(confidence[i])[:3]) * 255).astype(int).tolist())
    cv2.line(canvas, (int(kp0[0]), int(kp0[1])), (int(kp1[0]), int(kp1[1])), color, 2)

  return canvas


def main():
  parser = argparse.ArgumentParser(description='Initial guess matching based on DaD keypoints and RoMa', formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('data_path', help='Input data path')
  parser.add_argument('--num_keypoints', type=int, default=2000, help='Number of DaD keypoints detected in each image')
  parser.add_argument('--coarse_res', type=int, default=560, help='RoMa coarse matching resolution (multiple of 14)')
  parser.add_argument('--upsample_res', type=int, nargs=2, default=[864, 1152], metavar=('H', 'W'), help='RoMa refinement resolution (multiples of 8)')
  parser.add_argument('--max_dist', type=float, default=0.005, help='Max distance between a warped camera keypoint and its LiDAR keypoint (normalized image coordinates, 2 = image width)')
  parser.add_argument('--certainty_thresh', type=float, default=0.0, help='Min RoMa certainty of a match')
  parser.add_argument('--show_keypoints', action='store_true', help='Show the matches')
  parser.add_argument('--force_cpu', action='store_true', help='Force pytorch to run in CPU mode.')
  parser.add_argument('--rotate_camera', type=int, default=0, help='Rotate camera image before matching (CW 0, 90, 180, or 270) (CW)')
  parser.add_argument('--rotate_lidar', type=int, default=0, help='Rotate LiDAR image before matching (0, 90, 180, or 270) (CW)')

  opt = parser.parse_args()
  print(opt)

  torch.set_grad_enabled(False)
  device = torch.device('cuda' if torch.cuda.is_available() and not opt.force_cpu else 'cpu')
  print('Running inference on device \"{}\"'.format(device))

  data_path = opt.data_path
  with open(data_path + '/calib.json', 'r') as f:
    calib_config = json.load(f)

  matcher, detector = load_models(opt, device)

  for bag_name in calib_config['meta']['bag_names']:
    print('processing %s' % bag_name)

    camera_image = cv2.imread('%s/%s.png' % (data_path, bag_name), 0)
    lidar_image = cv2.imread('%s/%s_lidar_intensities.png' % (data_path, bag_name), 0)

    if opt.rotate_camera:
      code, camera_R_inv = angle_to_rot(opt.rotate_camera, camera_image.shape)
      camera_image = cv2.rotate(camera_image, code)
    if opt.rotate_lidar:
      code, lidar_R_inv = angle_to_rot(opt.rotate_lidar, lidar_image.shape)
      lidar_image = cv2.rotate(lidar_image, code)

    kpts0, kpts1, matches, confidence = match_images(matcher, detector, camera_image, lidar_image, opt, device)
    print('%d / %d keypoints matched' % (numpy.count_nonzero(matches >= 0), len(matches)))

    kpts0_ = kpts0
    kpts1_ = kpts1
    if opt.rotate_camera:
      kpts0_ = camera_R_inv(kpts0_)
    if opt.rotate_lidar:
      kpts1_ = lidar_R_inv(kpts1_)

    # initial_guess_auto reads integer pixel coordinates
    result = {
      'kpts0': numpy.rint(kpts0_).astype(int).flatten().tolist(),
      'kpts1': numpy.rint(kpts1_).astype(int).flatten().tolist(),
      'matches': matches.tolist(),
      'confidence': confidence.tolist(),
    }
    with open('%s/%s_matches.json' % (data_path, bag_name), 'w') as f:
      json.dump(result, f)

    # visualization (in the matched, possibly rotated, images)
    canvas = draw_matches(camera_image, lidar_image, kpts0, kpts1, matches, confidence)
    cv2.imwrite('%s/%s_matches.png' % (data_path, bag_name), canvas)

    if opt.show_keypoints:
      cv2.imshow('canvas', canvas)
      cv2.waitKey(0)


if __name__ == '__main__':
  main()
