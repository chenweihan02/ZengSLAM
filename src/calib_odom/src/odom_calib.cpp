#include "../include/calib_odom/odom_calib.hpp"

// 设置数据，即多少数据计算一次
void OdomCalib::SetDataLen(int len) {
  data_len = len;
  A.conservativeResize(len * 3, 9);
  b.conservativeResize(len * 3);
  A.setZero();
  b.setZero();
}

/*
输入：里程计和激光数据

TODO:
构建最小二乘需要的超定方程组
Ax = b

*/              
bool OdomCalib::AddData(Eigen::Vector3d odom, Eigen::Vector3d scan) {
  if (now_len < INT_MAX) {
    //　构建超定方程组
    A(now_len % data_len * 3, 0) = odom(0);
    A(now_len % data_len * 3, 1) = odom(1);
    A(now_len % data_len * 3, 2) = odom(2);
    A(now_len % data_len * 3 + 1, 3) = odom(0);
    A(now_len % data_len * 3 + 1, 4) = odom(1);
    A(now_len % data_len * 3 + 1, 5) = odom(2);
    A(now_len % data_len * 3 + 2, 6) = odom(0);
    A(now_len % data_len * 3 + 2, 7) = odom(1);
    A(now_len % data_len * 3 + 2, 8) = odom(2);

    b(now_len % data_len * 3) = scan(0);
    b(now_len % data_len * 3 + 1) = scan(1);
    b(now_len % data_len * 3 + 2) = scan(2);

    now_len ++;
    return true;
  } else {
    return false;
  }
}

/*
用于判断数据是否满
数据满即可进行最小二乘计算
*/
bool OdomCalib::IsFull() {
  if (now_len % data_len == 0 && now_len >= 1) {
    now_len = data_len;
    return true;
  } else {
    return false;
  }
}

/*
解最小二乘
返回最小二乘矩阵
*/
Eigen::Matrix3d OdomCalib::Solve() {
  Eigen::Matrix3d correct_matrix;

  //求解线性最小二乘
  Eigen::VectorXd correct_vector = A.colPivHouseholderQr().solve(b);
  
  correct_matrix << correct_vector(0), correct_vector(1), correct_vector(2),
                    correct_vector(3), correct_vector(4), correct_vector(5),
                    correct_vector(6), correct_vector(7), correct_vector(8);
  
  return correct_matrix;
}

/*
数据清零
*/
void OdomCalib::SetDataZero() {
  A.setZero();
  b.setZero();
}