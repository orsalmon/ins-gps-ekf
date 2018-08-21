//
// Created by Or Salmon on 21/07/18.
//

#include "KittiDatasetTools/KittiDatasetParser.h"
#include <thread>

namespace kitti_dataset_tools {
KittiDatasetParser::KittiDatasetParser(std::string path_to_oxts_folder)
    : path_to_oxts_folder_(path_to_oxts_folder), is_playing_(false),
      is_data_ready_(false) {}

bool KittiDatasetParser::loadDataset() {
  if (!loadTimestamp()) {
    std::cout << "Failed to load Kitti timestamp file." << std::endl;
    return false;
  }
  if (!loadData()) {
    std::cout << "Failed to load Kitti data files." << std::endl;
    return false;
  }
  is_data_ready_ = true;
  return true;
}

bool KittiDatasetParser::loadTimestamp() {
  std::ifstream import_file(path_to_oxts_folder_ + "timestamps.txt",
                            std::ios::in);
  if (!import_file) {
    return false;
  }

  timestamp_vec_.clear();
  std::string line;
  while (std::getline(import_file, line)) {
    std::stringstream line_stream(line);

    std::string timestamp_string = line_stream.str();
    std::tm t = {};
    t.tm_year = std::stoi(timestamp_string.substr(0, 4)) - 1900;
    t.tm_mon = std::stoi(timestamp_string.substr(5, 2)) - 1;
    t.tm_mday = std::stoi(timestamp_string.substr(8, 2));
    t.tm_hour = std::stoi(timestamp_string.substr(11, 2));
    t.tm_min = std::stoi(timestamp_string.substr(14, 2));
    t.tm_sec = std::stoi(timestamp_string.substr(17, 2));
    t.tm_isdst = -1;

    static const uint64_t kSecondsToNanoSeconds = 1e9;
    time_t time_since_epoch = mktime(&t);

    uint64_t timestamp = time_since_epoch * kSecondsToNanoSeconds +
                         std::stoi(timestamp_string.substr(20, 9));
    timestamp_vec_.push_back(timestamp);
  }

  std::cout << "Timestamps: " << std::endl
            << "from: " << timestamp_vec_.front()
            << " to: " << timestamp_vec_.back() << std::endl;
  return true;
}

bool KittiDatasetParser::loadData() {
  const auto data_length = timestamp_vec_.size();

  gps_vec_ = new Eigen::MatrixXd(data_length, 6);
  accelerometer_vec_ = new Eigen::MatrixXd(data_length, 3);
  gyro_vec_ = new Eigen::MatrixXd(data_length, 3);

  for (int i = 0; i < data_length; ++i) {
    std::string filename = std::string(9, '0').append(std::to_string(i));
    if (filename.length() > 10)
      filename.erase(0, filename.length() - 10);
    std::ifstream import_file(
        path_to_oxts_folder_ + "data/" + filename + ".txt", std::ios::in);
    if (!import_file) {
      return false;
    }

    std::string data;
    getline(import_file, data);
    std::vector<std::string> data_vec;

    size_t pos = 0;
    std::string token;
    std::string delimiter = " ";
    while ((pos = data.find(delimiter)) != std::string::npos) {
      token = data.substr(0, pos);
      data_vec.push_back(token);
      data.erase(0, pos + delimiter.length());
    }
    data_vec.push_back(token);

    (*gps_vec_)(i, 0) = std::stod(data_vec.at(lat));
    (*gps_vec_)(i, 1) = std::stod(data_vec.at(lon));
    (*gps_vec_)(i, 2) = std::stod(data_vec.at(alt));
    (*gps_vec_)(i, 3) = std::stod(data_vec.at(vn));
    (*gps_vec_)(i, 4) = std::stod(data_vec.at(ve));
    (*gps_vec_)(i, 5) = -std::stod(data_vec.at(vu));

    (*accelerometer_vec_)(i, 0) = std::stod(data_vec.at(ax));
    (*accelerometer_vec_)(i, 1) = std::stod(data_vec.at(ay));
    (*accelerometer_vec_)(i, 2) = std::stod(data_vec.at(az));

    (*gyro_vec_)(i, 0) = std::stod(data_vec.at(wx));
    (*gyro_vec_)(i, 1) = std::stod(data_vec.at(wy));
    (*gyro_vec_)(i, 2) = std::stod(data_vec.at(wz));
  }

  return true;
}

void KittiDatasetParser::startPlayingData(EKF_INS::EKF &ekf) {
  if (!is_data_ready_) {
    std::cout << "Can't play. Data is not ready." << std::endl;
    return;
  }
  Eigen::Vector3d p_0((*gps_vec_)(0, 0), (*gps_vec_)(0, 1), (*gps_vec_)(0, 2));
  ekf.setInitialState(p_0, Eigen::Vector3d::Zero(),
                      Eigen::Matrix3d::Identity());
  ekf.start();
  for (int i = 1; i < timestamp_vec_.size(); ++i) {
    Eigen::Vector3d acc((*accelerometer_vec_)(i, 0),
                        (*accelerometer_vec_)(i, 1),
                        (*accelerometer_vec_)(i, 2));
    Eigen::Vector3d gyro((*gyro_vec_)(i, 0), (*gyro_vec_)(i, 1),
                         (*gyro_vec_)(i, 2));
    ekf.updateWithInertialMeasurement(acc, EKF_INS::Type::ACCELEROMETER);
    ekf.updateWithInertialMeasurement(gyro, EKF_INS::Type::GYRO);

    std::vector<Eigen::Matrix<double, 6, 1>> gps_data;
    gps_data.push_back((*gps_vec_).row(i).transpose());
    ekf.updateWithGPSMeasurements(gps_data);

    uint64_t dt = (timestamp_vec_.at(i) - timestamp_vec_.at(i-1));
    std::this_thread::sleep_for(std::chrono::duration<uint64_t , std::nano>(dt));
  }
}
} // namespace kitti_dataset_tools