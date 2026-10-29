# Program details

![Screenshot_20230119_173747](https://user-images.githubusercontent.com/31344317/213393928-c640e0d8-076a-4272-90b3-c67dfab02358.png){: style="height:75%;width:75%"}

## 1. Preprocessing

```bash
$ preprocess data_path dst_path
```

`data_path` holds one directory per sequence with an image, `camera_info.json` and a folder of PCD/PLY scans ([dataset format](https://github.com/kolton-es/direct_visual_lidar_calibration#dataset-format)); convert ROS bags with `bag_to_dataset.py BAGS --out data_path`.

| Option                             | Description                                                                     |
|:-----------------------------------|:--------------------------------------------------------------------------------|
| -v [ --visualize ]                 | Show points accumulation process                                                |
| -a [ --auto_topic ]                | Select image/camera_info/points automatically (the default; kept for compatibility) |
| -d [ --dynamic_lidar_integration ] | Create target point clouds from dynamic LiDAR data (for velodyne-like LiDARs)   |
| -i [ --intensity_channel ]         | "auto" or point intensity channel name                                          |
| --camera_info                      | Camera info file in each sequence (default: the only `*camera_info*.json`)      |
| --image                            | Image file in each sequence (default: the only image)                           |
| --points                           | Scan directory in each sequence (default: the only directory of PCD/PLY files)  |
| --camera_model                     | auto, atan, plumb_bob, fisheye, omnidir, or equirectangular                     |
| --camera_intrinsic                 | Camera intrinsic parameters: fx,fy,cx,cy(,xi) (Don't put spaces between values) |
| --camera_distortion_coeffs         | Camera distortion parameters [k1,k2,p1,p2,k3] (Don't put spaces between values) |
| --voxel_resolution                 | Downsampling resolution                                                         |
| --min_distance                     | Minimum point distance. Points closer than this value will be discarded         |

***Example1***: Use the *dynamic point integrator* for an Ouster LiDAR
```bash
# -d = Dynamic points integration
# -v = Visualization
$ preprocess ouster_pinhole ouster_pinhole_preprocessed -d -v
```

!!!note
    Automatic selection requires each sequence to contain a single image, camera_info file and scan directory.

!!!note
    ```distortion_model``` in camera_info.json must be ```plumb_bob``` or ```fisheye```. For other distortion models, specify the camera model name and parameters manually as in Example2.


***Example2***: Manually specify the image, the scans and camera parameters
```
$ preprocess ouster_pinhole ouster_pinhole_preprocessed \
  --image image.png \
  --points points \
  --camera_model plumb_bob \
  --camera_intrinsics 810.38,810.28,822.84,622.47
  --camera_distortion_coeffs
```

## 2. Initial guess

### Option1: Manual estimation

```bash
$ initial_guess_manual preprocessed_data_path
```

1. Right click a 3D point on the point cloud and a corresponding 2D point on the image
2. Click ```Add picked points``` button
3. Repeat 1 and 2 for several points (At least three points. The more the better.)
4. Click ```Estimate``` button to obtain an initial guess of the LiDAR-camera transformation
5. Check if the image projection result is fine by changing ```blend_weight```
6. Click ```Save``` button to save the initial guess

<div class="youtube">
<iframe width="560" height="315" src="https://www.youtube.com/embed/FTlC9RwEVxY" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>
</div>

### Option2: Automatic estimation

```bash
$ find_matches_roma.py preprocessed_data_path
$ initial_guess_auto preprocessed_data_path
```

`find_matches_roma.py` detects [DaD](https://github.com/Parskatt/dad) keypoints in the camera and LiDAR intensity images and matches them with the [RoMa](https://github.com/Parskatt/RoMa) dense matcher (both MIT licensed; this replaces upstream's SuperGlue, which is non-commercial). `initial_guess_auto` then estimates the transformation from the matches with RANSAC. Main options: `--num_keypoints` (DaD keypoints per image, default 2000), `--max_dist` (how close a warped camera keypoint must land to a LiDAR keypoint), `--certainty_thresh` (minimum RoMa certainty) and `--force_cpu`. It runs on the CPU in Docker (about 20 s per sequence) and on CUDA when available.

!!!note
    Matching works best when the upward directions of the camera and LiDAR images are roughly aligned. Use ```--rotate_camera``` and ```--rotate_lidar``` options to ensure this. (e.g., ```--rotate_camera 90```. Only 90, 180, and 270 degree rotations are allowed.)

!!!note
    You can find ```*_matches.png``` saved in the data directory that represents detected 2D-3D correspondences.



## 3. Calibration

```bash
$ calibrate preprocessed_data_path
```

| Option                                              | Description                                                                     |
|:----------------------------------------------------|:--------------------------------------------------------------------------------|
| --registration_type (default = nid_bfgs)            | ```nid_bfgs``` or ```nid_nelder_mead```                                         |
| --nid_bins (default = 16)                           | Number of histogram bins for NID                                                |
| --nelder_mead_init_step (default = 0.001)           | Nelder-mead initial step size                                                   |
| --nelder_mead_convergence_criteria (default = 1e-8) | Nelder-mead convergence criteria                                                |
| --auto_quit                                         | Automatically close the viewer after calibration                                |
| --background                                        | Disable visualization                                                           |

Once the calibration is completed, open ```calib.json``` in the data directory with a text editor and find the calibration result ```T_lidar_camera: [x, y, z, qx, qy, qz, qw]``` that transforms a 3D point in the camera frame into the LiDAR frame (i.e., ```p_lidar = T_lidar_camera * p_camera```).

```calib.json``` also contains camera parameters, manual/automatic initial guess results (```init_T_lidar_camera``` and ```init_T_lidar_camera_auto```), and some meta data.

<details>
  <summary>calib.json example</summary>

```json
{
  "camera": {
    "camera_model": "plumb_bob",
    "distortion_coeffs": [
      -0.0408800300227048,
      0.08232065129613146,
      0.0001524417339184569,
      -0.0002905086459989649,
      -0.03955344846871078
    ],
    "intrinsics": [
      810.3829359698531,
      810.2790141092258,
      822.8441591172331,
      622.4745298743934
    ]
  },
  "meta": {
    "bag_names": [
      "rosbag2_2022_12_09-11_51_00",
      "rosbag2_2022_12_09-11_51_39",
      "rosbag2_2022_12_09-11_52_13",
      "rosbag2_2022_12_09-11_52_50",
      "rosbag2_2022_12_09-11_53_36"
    ],
    "camera_info": "camera_info.json",
    "data_path": "ouster_pinhole",
    "image": "image.png",
    "intensity_channel": "reflectivity",
    "points": "points"
  },
  "results": {
    "T_lidar_camera": [
      0.029965406350829532,
      0.0018510163746144406,
      0.10836834957603067,
      -0.5020970411416648,
      0.49250979122625377,
      -0.5009468032383634,
      0.5043659060130069
    ],
    "init_T_lidar_camera": [
      0.22754979133605957,
      0.14180368185043335,
      0.09482517838478088,
      -0.4999842630484326,
      0.4931746122747289,
      -0.5037605589800136,
      0.5030107918869041
    ],
    "init_T_lidar_camera_auto": [
      -0.06012828990854561,
      0.03957544424313349,
      0.09638527740996672,
      -0.5015219498041896,
      0.4932277405168562,
      -0.5021955602061449,
      0.5029927923524125
    ]
  }
}
```

</details>

!!!tip
    If you need the transformation in another form (e.g., 4x4 matrix), use the [Matrix converter](https://staff.aist.go.jp/k.koide/workspace/matrix_converter/matrix_converter.html).

## 4. Result inspection

```bash
$ viewer preprocessed_data_path
```
![Screenshot_20230119_173547](https://user-images.githubusercontent.com/31344317/213393507-efe30dce-097f-4b65-b91f-56336454991d.png)