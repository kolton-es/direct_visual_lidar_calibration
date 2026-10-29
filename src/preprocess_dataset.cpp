#include <vlcal/preprocess/preprocess.hpp>

#include <set>
#include <fstream>
#include <iostream>
#include <algorithm>
#include <filesystem>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/io/auto_io.h>
#include <nlohmann/json.hpp>

#include <vlcal/common/cloud_converter.hpp>
#include <vlcal/common/console_colors.hpp>

// Reads a dataset folder instead of rosbags. Each sequence (the counterpart of
// one rosbag) is a directory holding one camera image, an optional camera info
// file and a folder of LiDAR scans, one PCD or PLY file per scan named by its
// timestamp in integer nanoseconds:
//
//   data_path/
//     sequence_a/
//       image.png              (PNG, JPEG, BMP or TIFF)
//       camera_info.json       {"distortion_model", "intrinsics": [fx,fy,cx,cy], "distortion_coeffs": [...]}
//       points/1679988354559472320.pcd
//       points/1679988354659472320.pcd
//       ...
//
// The "topics" of the Preprocess interface are paths relative to each sequence
// directory (image.png, camera_info.json, points).

namespace vlcal {

namespace fs = std::filesystem;

namespace {

std::string lower_extension(const fs::path& path) {
  std::string ext = path.extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
  return ext;
}

bool is_image_file(const fs::path& path) {
  static const std::set<std::string> exts = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"};
  return fs::is_regular_file(path) && exts.count(lower_extension(path));
}

bool is_cloud_file(const fs::path& path) {
  static const std::set<std::string> exts = {".pcd", ".ply"};
  return fs::is_regular_file(path) && exts.count(lower_extension(path));
}

/// Scan files in a points directory, sorted by their integer nanosecond timestamps
std::vector<std::pair<std::int64_t, std::string>> list_scans(const fs::path& points_dir) {
  std::vector<std::pair<std::int64_t, std::string>> scans;
  if (!fs::is_directory(points_dir)) {
    return scans;
  }

  for (const auto& entry : fs::directory_iterator(points_dir)) {
    if (!is_cloud_file(entry.path())) {
      continue;
    }

    const std::string stem = entry.path().stem().string();
    std::size_t pos = 0;
    std::int64_t stamp = 0;
    try {
      stamp = std::stoll(stem, &pos);
    } catch (const std::exception&) {
      pos = 0;
    }
    if (pos != stem.size()) {
      std::cerr << console::bold_red << "error: scan file name is not an integer nanosecond timestamp: " << entry.path() << console::reset << std::endl;
      abort();
    }
    scans.emplace_back(stamp, entry.path().string());
  }

  std::sort(scans.begin(), scans.end());
  return scans;
}

pcl::PCLPointCloud2 load_cloud(const std::string& filename) {
  pcl::PCLPointCloud2 cloud;
  if (pcl::io::load(filename, cloud) != 0) {
    std::cerr << console::bold_red << "error: failed to load " << filename << console::reset << std::endl;
    abort();
  }
  return cloud;
}

}  // namespace

class PointCloudReaderDataset : public PointCloudReader {
public:
  PointCloudReaderDataset(const fs::path& points_dir, const std::string& intensity_channel)
  : intensity_channel(intensity_channel),
    scans(list_scans(points_dir)),
    next(0) {}

  virtual RawPoints::Ptr read_next() override {
    while (next < scans.size()) {
      const auto& [stamp, filename] = scans[next++];
      auto raw_points = extract_raw_points(load_cloud(filename), stamp / 1e9, intensity_channel);
      if (raw_points) {
        return raw_points;
      }
      std::cerr << console::yellow << "warning: skip " << filename << console::reset << std::endl;
    }
    return nullptr;
  }

private:
  const std::string intensity_channel;
  const std::vector<std::pair<std::int64_t, std::string>> scans;
  std::size_t next;
};

class PreprocessDataset : public Preprocess {
protected:
  virtual bool valid_bag(const std::string& bag_filename) override {
    const fs::path dir(bag_filename);
    if (!fs::is_directory(dir)) {
      return false;
    }

    for (const auto& entry : fs::directory_iterator(dir)) {
      if (entry.is_directory() && !list_scans(entry.path()).empty()) {
        return true;
      }
    }
    return false;
  }

  virtual std::vector<std::pair<std::string, std::string>> get_topics_and_types(const std::string& bag_filename) override {
    std::vector<fs::path> entries;
    for (const auto& entry : fs::directory_iterator(bag_filename)) {
      entries.emplace_back(entry.path());
    }
    std::sort(entries.begin(), entries.end());

    std::vector<std::pair<std::string, std::string>> topics_and_types;
    for (const auto& path : entries) {
      const std::string name = path.filename().string();
      if (is_image_file(path)) {
        topics_and_types.emplace_back(name, "Image");
      } else if (fs::is_regular_file(path) && lower_extension(path) == ".json" && name.find("camera_info") != std::string::npos) {
        topics_and_types.emplace_back(name, "CameraInfo");
      } else if (fs::is_directory(path) && !list_scans(path).empty()) {
        topics_and_types.emplace_back(name, "PointCloud2");
      }
    }

    return topics_and_types;
  }

  virtual std::vector<std::string> get_point_fields(const std::string& bag_filename, const std::string& points_topic) override {
    const auto scans = list_scans(fs::path(bag_filename) / points_topic);
    if (scans.empty()) {
      std::cerr << console::bold_red << "error: no scans in " << (fs::path(bag_filename) / points_topic) << console::reset << std::endl;
      abort();
    }

    const auto cloud = load_cloud(scans.front().second);
    std::vector<std::string> fields(cloud.fields.size());
    std::transform(cloud.fields.begin(), cloud.fields.end(), fields.begin(), [](const auto& field) { return field.name; });
    return fields;
  }

  virtual cv::Size get_image_size(const std::string& bag_filename, const std::string& image_topic) override {
    return get_image(bag_filename, image_topic).size();
  }

  virtual std::tuple<std::string, std::vector<double>, std::vector<double>> get_camera_info(const std::string& bag_filename, const std::string& camera_info_topic) override {
    const fs::path path = fs::path(bag_filename) / camera_info_topic;
    std::ifstream ifs(path);
    if (camera_info_topic.empty() || !ifs) {
      std::cerr << console::bold_red << "error: failed to open camera info " << path << console::reset << std::endl;
      std::cerr << console::bold_red << "     : add camera_info.json or specify --camera_model, --camera_intrinsics and --camera_distortion_coeffs" << console::reset
                << std::endl;
      abort();
    }

    const auto camera_info = nlohmann::json::parse(ifs);
    const std::string distortion_model = camera_info["distortion_model"];
    const std::vector<double> intrinsics = camera_info["intrinsics"];
    const std::vector<double> distortion_coeffs = camera_info.value("distortion_coeffs", std::vector<double>());
    return {distortion_model, intrinsics, distortion_coeffs};
  }

  virtual cv::Mat get_image(const std::string& bag_filename, const std::string& image_topic) override {
    const fs::path path = fs::path(bag_filename) / image_topic;
    cv::Mat image = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (!image.data) {
      std::cerr << console::bold_red << "error: failed to load image " << path << console::reset << std::endl;
      abort();
    }

    // Same conversions as cv_bridge to mono8
    if (image.depth() == CV_16U) {
      image.convertTo(image, CV_8U, 1.0 / 256.0);
    }
    if (image.channels() == 3) {
      cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    } else if (image.channels() == 4) {
      cv::cvtColor(image, image, cv::COLOR_BGRA2GRAY);
    }
    return image;
  }

  virtual std::shared_ptr<PointCloudReader> get_point_cloud_reader(const std::string& bag_filename, const std::string& points_topic, const std::string& intensity_channel)
    override {
    return std::make_shared<PointCloudReaderDataset>(fs::path(bag_filename) / points_topic, intensity_channel);
  }
};

}  // namespace vlcal

int main(int argc, char** argv) {
  vlcal::PreprocessDataset preprocess;
  return preprocess.run(argc, argv) ? 0 : 1;
}
