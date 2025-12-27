
#include <Eigen/Dense>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>
using namespace std;

// 规范
// 如果你已经知道了一个简单的变换关系，T_a_to_b_ 应该是 b 坐标系在 a 坐标系下的表示
// T_a_to_b_: Transform from 'a' coordinate frame to 'b' frame
// 点变换: P_b = T_a_to_b_ * P_a
// 点逆变换: P_a = invert_transform(T_a_to_b_) * P_b
// 位姿变换: T_b = T_a_to_b_ * T_a
// 位姿逆变换: T_a = invert_transform(T_a_to_b_) * T_b

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

// 规整rpy的输出，以更加贴近参考值的形式展示
Eigen::Vector3d get_rpy(const Eigen::Vector3d &rpy_in, const Eigen::Affine3d& pose)
{
    Eigen::Vector3d rpy = rpy_in;
    Eigen::Matrix3d R = pose.linear();
    Eigen::Vector3d z_axis = R.col(2);

    const double eps = 1e-5;

    // 正向竖直向上
    if (std::abs(z_axis.z() - 1.0) < eps) {
        rpy.x() = 0.0;
        rpy.y() = 0.0;
        rpy.z() = 0.0;  // yaw 规整到 0
    }
    // 反向竖直向下
    else if (std::abs(z_axis.z() + 1.0) < eps) {
        rpy.x() = M_PI;
        rpy.y() = 0.0;
        rpy.z() = 0.0;  // yaw 规整到 0
    }

    // 可以加一行规整 yaw 到 [-pi, pi]
    rpy.z() = std::atan2(std::sin(rpy.z()), std::cos(rpy.z()));

    return rpy;
}

// Helper functions
// 如果生成的是T_a_to_b_，则translation和rpy均为b坐标系在a坐标系下的表示
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

// void test0()
// {
//   auto T_base_link_to_base_link_left_ = make_transform_matrix({0, 0, 0}, {0, 0, -M_PI / 6});

//   // 1. test point transform
//   Eigen::Vector3d P_base_link_x(1, 0, 0);  // point in base_link frame
//   Eigen::Vector3d P_base_link_left_x = T_base_link_to_base_link_left_ * P_base_link_x;
//   cout << "P_base_link_left: " << P_base_link_left_x.transpose() << endl;
//   Eigen::Vector3d P_base_link_y(0, 1, 0);  // point in base_link frame
//   Eigen::Vector3d P_base_link_left_y = T_base_link_to_base_link_left_ * P_base_link_y;
//   cout << "P_base_link_left: " << P_base_link_left_y.transpose() << endl;
//   Eigen::Vector3d P_base_link_z(0, 0, 1);  // point in base_link frame
//   Eigen::Vector3d P_base_link_left_z = T_base_link_to_base_link_left_ * P_base_link_z;
//   cout << "P_base_link_left: " << P_base_link_left_z.transpose() << endl;

//   // 2.inverse transform test
//   Eigen::Vector3d P_base_link_x_back = invert_transform(T_base_link_to_base_link_left_) * P_base_link_left_x;
//   cout << "P_base_link: " << P_base_link_x_back.transpose() << endl;
//   Eigen::Vector3d P_base_link_y_back = invert_transform(T_base_link_to_base_link_left_) * P_base_link_left_y;
//   cout << "P_base_link: " << P_base_link_y_back.transpose() << endl;
//   Eigen::Vector3d P_base_link_z_back = invert_transform(T_base_link_to_base_link_left_) * P_base_link_left_z;
//   cout << "P_base_link: " << P_base_link_z_back.transpose() << endl;

//   // 3. transform test
//   auto T_fl_world_ = make_transform_matrix({0, 0, 0}, {0, 0, 0});
//   print_pose(T_fl_world_, "T_fl_world_");
//   auto T_lt_world_ = T_fl_world_ * T_base_link_to_base_link_left_;
//   print_pose(T_lt_world_, "T_lt_world_");

//   // 4. inverse transform test
//   auto T_fl_world_back = T_lt_world_ * invert_transform(T_base_link_to_base_link_left_);
//   print_pose(T_fl_world_back, "T_fl_world_back");
// }

void test1()
{
  auto T_forward_lt_to_lt_ = make_transform_matrix({0, 0, 0}, {0, -M_PI / 2, -M_PI / 2});

  // 1. test point transform
  Eigen::Vector3d P_forward_lt_x(1, 0, 0);  // point in forward_lt frame
  Eigen::Vector3d P_lt_x = T_forward_lt_to_lt_ * P_forward_lt_x;
  cout << "P_lt: " << P_lt_x.transpose() << endl;
  Eigen::Vector3d P_forward_lt_y(0, 1, 0);  // point in forward_lt frame
  Eigen::Vector3d P_lt_y = T_forward_lt_to_lt_ * P_forward_lt_y;
  cout << "P_lt: " << P_lt_y.transpose() << endl;
  Eigen::Vector3d P_forward_lt_z(0, 0, 1);  // point in forward_lt frame
  Eigen::Vector3d P_lt_z = T_forward_lt_to_lt_ * P_forward_lt_z;
  cout << "P_lt: " << P_lt_z.transpose() << endl;
  cout << endl;
  // 2.inverse transform test
  Eigen::Vector3d P_forward_lt_x_back = invert_transform(T_forward_lt_to_lt_) * P_lt_x;
  cout << "P_forward_lt: " << P_forward_lt_x_back.transpose() << endl;
  Eigen::Vector3d P_forward_lt_y_back = invert_transform(T_forward_lt_to_lt_) * P_lt_y;
  cout << "P_forward_lt: " << P_forward_lt_y_back.transpose() << endl;
  Eigen::Vector3d P_forward_lt_z_back = invert_transform(T_forward_lt_to_lt_) * P_lt_z;
  cout << "P_forward_lt: " << P_forward_lt_z_back.transpose() << endl;
  cout << endl;
  // 3. transform test
  auto pose_in_forward_lt = make_transform_matrix({0, 0, 0}, {0, 0, 0});
  print_pose(pose_in_forward_lt, "pose_in_forward_lt");
  auto pose_in_lt = T_forward_lt_to_lt_ * pose_in_forward_lt;
  print_pose(pose_in_lt, "pose_in_lt");
  cout << endl;
  // 4. inverse transform test
  auto pose_in_forward_lt_back = invert_transform(T_forward_lt_to_lt_) * pose_in_lt;
  print_pose(pose_in_forward_lt_back, "pose_in_forward_lt_back");
  cout << endl;
}

// void test2()
// {
//   auto T_vr_to_human_align_ = make_transform_matrix({0, 0, 0}, {0, -M_PI * 35 / 180, 0});  // -35 degrees
//   auto pose_in_vr = make_transform_matrix({0, 0, 0}, {0, M_PI * 35 / 180, 0});
//   auto pose_in_human = T_vr_to_human_align_ * pose_in_vr;
//   print_pose(pose_in_human, "pose_in_human");
// }

int main()
{
  test1();
  return 0;
}