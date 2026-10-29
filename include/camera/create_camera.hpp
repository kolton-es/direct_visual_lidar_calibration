#pragma once

#include <memory>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>
#include <camera/generic_camera_base.hpp>

namespace camera {

/**
 * @brief Create a generic camera model
 * @param camera_model       Camera projection model (plumb_bob, fisheye, atan, omnidir, or equirectangular)
 * @param intrinsics         Camera intrinsic parameters
 * @param distortion_coeffs  Camera distortion coefficients
 */
camera::GenericCameraBase::ConstPtr create_camera(const std::string& camera_model, const std::vector<double>& intrinsics, const std::vector<double>& distortion_coeffs);

/**
 * @brief Load a camera given by a precomputed pixel remap (see camera/remap.hpp)
 * @param remap_file  OpenCV FileStorage file (.yml, .yaml, .xml, .json, optionally .gz)
 *                    with image_width, image_height, virtual_model (equidistant),
 *                    virtual_fx, virtual_fy, virtual_cx, virtual_cy and map (CV_32FC2)
 * @return nullptr on failure
 */
camera::GenericCameraBase::ConstPtr load_remap_camera(const std::string& remap_file);

/**
 * @brief Create the camera of a calib.json "camera" entry
 * @param camera_config  {"camera_model", "intrinsics", "distortion_coeffs"}, or
 *                       {"camera_model": "remap", "remap_file"} for a remap camera
 * @param data_path      Directory that a relative remap_file is relative to
 */
camera::GenericCameraBase::ConstPtr create_camera(const nlohmann::json& camera_config, const std::string& data_path);

}  // namespace vlcal
