//
// Created by Or Salmon on 21/07/18.
//

#include "KittiDatasetTools/KittiDatasetParser.h"
#include "EKF.h"
#include "matplotlib-cpp/matplotlibcpp.h"
#include <boost/asio.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>

int main(int argc, char **argv) {
  EKF_INS::EKF ekf;
  std::string path = "/home/or-caja/Downloads/oxts/";
  kitti_dataset_tools::KittiDatasetParser parser(path);
  parser.loadDataset();
  parser.startPlayingData(ekf);
  return 0;
}