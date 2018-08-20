//
// Created by Or Salmon on 21/07/18.
//

#ifndef EKF_INS_KITTIDATASETPARSER_H
#define EKF_INS_KITTIDATASETPARSER_H

#include "Core.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include "EKF.h"

namespace kitti_dataset_tools {
enum KittiDataFormat {
  lat,
  lon,
  alt,
  roll,
  pitch,
  yaw,
  vn,
  ve,
  vf,
  vl,
  vu,
  ax,
  ay,
  az,
  af,
  al,
  au,
  wx,
  wy,
  wz,
  wf,
  wl,
  wu,
  pos_accuracy,
  vel_accuracy,
  navstat,
  numsats,
  posmode,
  velmode,
  orimode
};
class KittiDatasetParser {
public:
  KittiDatasetParser(std::string path_to_oxts_folder);
  bool loadDataset();
  void startPlayingData(EKF_INS::EKF &ekf);

private:
  bool loadTimestamp();
  bool loadData();

  std::string path_to_oxts_folder_;
  bool is_playing_;
  bool is_data_ready_;
  std::vector<uint64_t> timestamp_vec_;
  Eigen::MatrixXd *gps_vec_;
  Eigen::MatrixXd *accelerometer_vec_, *gyro_vec_;
};
} // namespace kitti_dataset_tools

#endif // EKF_INS_KITTIDATASETPARSER_H
