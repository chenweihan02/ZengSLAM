#ifndef ODOM_CALIB_H
#define ODOM_CALIB_H
#include <iostream>
#include <eigen3/Eigen/Jacobi>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Householder>

class OdomCalib {
  public:
    OdomCalib() {
      data_len = 0;
      now_len = 0;
    }

  // virtual ~OdomCalib();
  void SetDataLen(int len);
  void AddData(Eigen::Vector3d odom, Eigen::Vector3d scan);
  Eigen::Matrix3d Solve();
  bool IsFull();
  void SetDataZero();


  private:
    Eigen::MatrixXd A;
    Eigen::MatrixXd b;
    int data_len, now_len;
};

#endif