
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
using namespace std;

Eigen::Affine3d invert_transform(const Eigen::Affine3d &T)
{
  return T.inverse();
}
void print_pose(const Eigen::Affine3d &T, const std::string &label)
{
  Eigen::Vector3d pos = T.translation();
  Eigen::Matrix3d rot = T.linear();
  Eigen::Vector3d rpy = rot.eulerAngles(0, 1, 2) * 180 / M_PI;  // degrees
  cout << label << " Position: x=" << pos.x() << ", y=" << pos.y() << ", z=" << pos.z() << endl;
  cout << label << " Orientation (rpy in deg): roll=" << rpy.x() << ", pitch=" << rpy.y() << ", yaw=" << rpy.z() << endl;
}
// Helper functions
Eigen::Affine3d make_transform_matrix(const std::vector<double> &translation, const std::vector<double> &rpy)
{
  Eigen::Affine3d T = Eigen::Affine3d::Identity();
  Eigen::Matrix3d R;
  // SciPy 'xyz' extrinsic: Rz * Ry * Rx
  R = Eigen::AngleAxisd(rpy[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(rpy[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(rpy[0], Eigen::Vector3d::UnitX());
  T.linear() = R;
  T.translation() = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  return T;
}

void test1()
{
  auto T_lt_forward_ = make_transform_matrix({0, 0, 0}, {0, -M_PI / 2, -M_PI / 2});
  Eigen::Vector3d test_forward(1, 0, 0);  // forward的x轴
  Eigen::Vector3d test_left(0, 1, 0);     // forward的y轴
  Eigen::Vector3d test_up(0, 0, 1);       // forward的z轴
  Eigen::Vector3d lt_x = T_lt_forward_.linear() * test_forward;
  Eigen::Vector3d lt_y = T_lt_forward_.linear() * test_left;
  Eigen::Vector3d lt_z = T_lt_forward_.linear() * test_up;
  cout << "T_lt_forward_ test: lt_x=(" << fixed << setprecision(3) << lt_x.x() << "," << lt_x.y() << "," << lt_x.z() << ")"
       << ", lt_y=(" << lt_y.x() << "," << lt_y.y() << "," << lt_y.z() << ")"
       << ", lt_z=(" << lt_z.x() << "," << lt_z.y() << "," << lt_z.z() << ")" << endl;
}

void test2()
{
  auto T_lt_forward_ = make_transform_matrix({0, 0, 0}, {0, -M_PI / 2, -M_PI / 2});
  auto T_fl_world_ = make_transform_matrix({0, 0, 0}, {0, 0, 0});
  auto T_lt_world_ = T_fl_world_ * T_lt_forward_;;
  print_pose(T_lt_world_, "T_lt_world_");
}
int main()
{
  test2();
  return 0;
}