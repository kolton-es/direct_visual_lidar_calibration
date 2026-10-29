#include <camera/create_camera.hpp>

#include <filesystem>
#include <ceres/jet.h>
#include <opencv2/core.hpp>

#include <camera/atan.hpp>
#include <camera/pinhole.hpp>
#include <camera/fisheye.hpp>
#include <camera/omnidir.hpp>
#include <camera/equirectangular.hpp>
#include <camera/rational_polynomial.hpp>
#include <camera/remap.hpp>
#include <camera/generic_camera.hpp>

#include <vlcal/common/console_colors.hpp>

namespace camera {

template <typename Projection>
camera::GenericCameraBase::ConstPtr create_camera(const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs) {
  if (intrinsics.size() != camera::CameraModelTraits<Projection>::num_intrinsic_params) {
    std::cerr << vlcal::console::bold_red << "error: num of intrinsic parameters mismatch!!" << vlcal::console::reset << std::endl;
    return nullptr;
  }

  std::vector<double> dist_coeffs(camera::CameraModelTraits<Projection>::num_distortion_params, 0.0);
  for (int i = 0; i < std::min(distortion_coeffs.size(), dist_coeffs.size()); i++) {
    dist_coeffs[i] = distortion_coeffs[i];
  }

  return std::make_shared<camera::GenericCamera<Projection>>(
    Eigen::Map<const Eigen::VectorXd>(intrinsics.data(), intrinsics.size()),
    Eigen::Map<const Eigen::VectorXd>(dist_coeffs.data(), dist_coeffs.size()));
}

camera::GenericCameraBase::ConstPtr create_camera(const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs) {
  if (camera_model == "plumb_bob") {
    return create_camera<camera::PinholeProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "fisheye" || camera_model == "equidistant") {
    return create_camera<camera::FisheyeProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "atan") {
    return create_camera<camera::ATANProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "omnidir") {
    return create_camera<camera::OmnidirectionalProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "equirectangular") {
    return create_camera<camera::EquirectangularProjection>(intrinsics, distortion_coeffs);
  } else if (camera_model == "rational_polynomial") {
    return create_camera<camera::RationalPolynomialProjection>(intrinsics, distortion_coeffs);
  }

  std::cerr << vlcal::console::bold_red << "error: unknown camera model " << camera_model << vlcal::console::reset << std::endl;
  return nullptr;
}

camera::GenericCameraBase::ConstPtr load_remap_camera(const std::string& remap_file) {
  const auto fail = [&](const std::string& message) -> camera::GenericCameraBase::ConstPtr {
    std::cerr << vlcal::console::bold_red << "error: " << message << " (" << remap_file << ")" << vlcal::console::reset << std::endl;
    return nullptr;
  };

  cv::FileStorage fs;
  try {
    if (!fs.open(remap_file, cv::FileStorage::READ)) {
      return fail("failed to open the camera remap");
    }
  } catch (const cv::Exception& e) {
    return fail(std::string("failed to read the camera remap: ") + e.what());
  }

  std::string virtual_model;
  fs["virtual_model"] >> virtual_model;
  if (virtual_model != "equidistant") {
    return fail("unsupported remap virtual_model '" + virtual_model + "' (expected equidistant)");
  }

  const int image_width = static_cast<int>(fs["image_width"]);
  const int image_height = static_cast<int>(fs["image_height"]);
  const Eigen::Vector4d virtual_intrinsics(
    static_cast<double>(fs["virtual_fx"]),
    static_cast<double>(fs["virtual_fy"]),
    static_cast<double>(fs["virtual_cx"]),
    static_cast<double>(fs["virtual_cy"]));

  cv::Mat map;
  fs["map"] >> map;
  if (map.empty() || map.channels() != 2 || image_width <= 0 || image_height <= 0 || !(virtual_intrinsics.head<2>().array() > 0.0).all()) {
    return fail("the camera remap needs image_width, image_height, virtual_fx/fy/cx/cy and a 2-channel map");
  }

  map.convertTo(map, CV_64FC2);
  std::vector<double> values(map.rows * map.cols * 2);
  for (int row = 0; row < map.rows; row++) {
    const double* src = map.ptr<double>(row);
    std::copy(src, src + map.cols * 2, values.begin() + row * map.cols * 2);
  }

  return std::make_shared<camera::RemapCamera>(virtual_intrinsics, map.cols, map.rows, std::move(values), Eigen::Vector2i(image_width, image_height));
}

camera::GenericCameraBase::ConstPtr create_camera(const nlohmann::json& camera_config, const std::string& data_path) {
  const std::string camera_model = camera_config["camera_model"];
  if (camera_model == "remap") {
    std::filesystem::path remap_file = camera_config["remap_file"].get<std::string>();
    if (remap_file.is_relative()) {
      remap_file = std::filesystem::path(data_path) / remap_file;
    }
    return load_remap_camera(remap_file.string());
  }

  const std::vector<double> intrinsics = camera_config["intrinsics"];
  const std::vector<double> distortion_coeffs = camera_config["distortion_coeffs"];
  return create_camera(camera_model, intrinsics, distortion_coeffs);
}

}  // namespace camera
