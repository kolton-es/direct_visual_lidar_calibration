#pragma once

#include <cmath>
#include <vector>
#include <Eigen/Core>
#include <ceres/jet.h>
#include <ceres/cubic_interpolation.h>
#include <camera/generic_camera_base.hpp>

namespace camera {

/**
 * @brief Camera given by a precomputed pixel remap instead of a parametric model.
 *
 * The remap is an OpenCV cv::remap map (CV_32FC2) from an ideal equidistant
 * camera (r = f * theta, no distortion; cv::fisheye with D = 0) to the real
 * camera: map(v', u') = the real pixel that sees the ray of virtual pixel
 * (u', v'). A point is projected analytically into the virtual image and the
 * map is interpolated bicubically there (differentiable, for autodiff), so any
 * calibrated model (e.g. Double Sphere) can be used through its map.
 *
 * Map entries that are not finite or below -1000 mark rays the source model
 * cannot project. Points that fall outside the map or next to such entries
 * project to invalid_pixel(), far outside any image.
 */
class RemapCamera : public GenericCameraBase {
public:
  /**
   * @param virtual_intrinsics  Equidistant camera [fx, fy, cx, cy] of the map
   * @param map_width           Map columns (virtual image width)
   * @param map_height          Map rows (virtual image height)
   * @param map                 Real pixel coordinates, row-major, interleaved [x, y]
   * @param image_size          Size of the real image [width, height]
   */
  RemapCamera(const Eigen::Vector4d& virtual_intrinsics, int map_width, int map_height, std::vector<double> map, const Eigen::Vector2i& image_size)
  : virtual_intrinsics(virtual_intrinsics),
    map_width(map_width),
    map_height(map_height),
    map(std::move(map)),
    image_size_(image_size),
    grid(this->map.data(), 0, map_height, 0, map_width),
    interpolator(grid) {
    // A cell is usable when the whole 4x4 bicubic stencil around it holds valid entries
    const auto valid = [&](int row, int col) {
      if (row < 0 || row >= map_height || col < 0 || col >= map_width) {
        return false;
      }
      const double* p = &this->map[2 * (row * map_width + col)];
      return std::isfinite(p[0]) && std::isfinite(p[1]) && p[0] > -1000.0 && p[1] > -1000.0;
    };

    cell_valid.assign(map_width * map_height, 0);
    for (int row = 0; row < map_height; row++) {
      for (int col = 0; col < map_width; col++) {
        bool ok = true;
        for (int dr = -1; dr <= 2 && ok; dr++) {
          for (int dc = -1; dc <= 2 && ok; dc++) {
            ok = valid(row + dr, col + dc);
          }
        }
        cell_valid[row * map_width + col] = ok;
      }
    }
  }

  // The interpolator keeps a pointer into map
  RemapCamera(const RemapCamera&) = delete;
  RemapCamera& operator=(const RemapCamera&) = delete;

  virtual Eigen::Vector2d project(const Eigen::Vector3d& point_3d) const override { return project_impl(point_3d); }

  virtual Eigen::Vector2d operator()(const Eigen::Vector3d& point_3d) const override { return project_impl(point_3d); }

  virtual Eigen::Matrix<ceres::Jet<double, 7>, 2, 1> operator()(const Eigen::Matrix<ceres::Jet<double, 7>, 3, 1>& point_3d) const override {
    return project_impl(point_3d);
  }

  /// Size of the real image [width, height]
  const Eigen::Vector2i& image_size() const { return image_size_; }

  static Eigen::Vector2d invalid_pixel() { return Eigen::Vector2d(-1e6, -1e6); }

private:
  static double real(double x) { return x; }
  template <int N>
  static double real(const ceres::Jet<double, N>& x) {
    return x.a;
  }

  template <typename T>
  Eigen::Matrix<T, 2, 1> project_impl(const Eigen::Matrix<T, 3, 1>& point_3d) const {
    using std::atan2;
    using std::sqrt;

    const Eigen::Matrix<T, 2, 1> invalid = invalid_pixel().cast<T>();

    // Equidistant projection into the virtual image: radius = f * theta
    const T& x = point_3d.x();
    const T& y = point_3d.y();
    const T& z = point_3d.z();
    const double r_real = std::hypot(real(x), real(y));

    T scale;  // theta / r
    if (r_real > 1e-9 * std::abs(real(z))) {
      const T r = sqrt(x * x + y * y);
      scale = atan2(r, z) / r;
    } else if (real(z) > 0.0) {
      scale = T(1.0) / z;  // on the optical axis, theta / r -> 1 / z
    } else {
      return invalid;
    }

    const T u = virtual_intrinsics[0] * scale * x + virtual_intrinsics[2];
    const T v = virtual_intrinsics[1] * scale * y + virtual_intrinsics[3];

    const double u_real = real(u);
    const double v_real = real(v);
    if (!std::isfinite(u_real) || !std::isfinite(v_real) || u_real < 0.0 || v_real < 0.0 || u_real >= map_width - 1 || v_real >= map_height - 1) {
      return invalid;
    }
    if (!cell_valid[static_cast<int>(v_real) * map_width + static_cast<int>(u_real)]) {
      return invalid;
    }

    Eigen::Matrix<T, 2, 1> pixel;
    interpolator.Evaluate(v, u, pixel.data());
    return pixel;
  }

private:
  const Eigen::Vector4d virtual_intrinsics;
  const int map_width;
  const int map_height;
  const std::vector<double> map;
  const Eigen::Vector2i image_size_;

  std::vector<char> cell_valid;
  ceres::Grid2D<double, 2> grid;
  ceres::BiCubicInterpolator<ceres::Grid2D<double, 2>> interpolator;
};

}  // namespace camera
