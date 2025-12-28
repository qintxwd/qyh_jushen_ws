#pragma once
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>

/**
 * @file qyh_pose3d.hpp
 * @brief QyhPose3D 和 QyhPoint3D 变换工具（参考 Cartographer Rigid3d 设计）
 *
 * 本文件提供 QyhPose3D（刚体变换）和 QyhPoint3D（点）类型，以及方便的变换运算。
 *
 * ----------------------------------------------------------------------
 * 规范说明：
 *
 * 1️⃣ 坐标系表示
 *    QyhPose3D 内部旋转使用 Eigen::Quaterniond，平移使用 Eigen::Vector3d。
 *    QyhPose3D 表示子坐标系在父坐标系下的刚体变换。
 *
 * 2️⃣ 点变换
 *    QyhPoint3D P_a;           // 在父坐标系 A 下的点
 *    QyhPoint3D P_b = T_a_to_b * P_a; // 转换到子坐标系 B
 *    点逆变换：P_a = T_a_to_b.inverse() * P_b;
 *
 * 3️⃣ 位姿变换
 *    QyhPose3D T_a;            // 位姿在父坐标系 A 下
 *    QyhPose3D T_b = T_a * T_a_to_b; // 转换到子坐标系 B
 *    位姿逆变换：T_a = T_b * T_a_to_b.inverse();
 *
 * 4️⃣ 旋转和物理一致性
 *    QyhPose3D 内部旋转与真实物理旋转一致，直接使用 * 运算符即可处理点和位姿。
 *
 * 5️⃣ 示例
 *    QyhPoint3D P_a{1,0,0};
 *    QyhPose3D T_a_to_b = QyhPose3D::FromRPY({0,0,M_PI/6});
 *    QyhPoint3D P_b = T_a_to_b * P_a; // 转换点
 *
 * ----------------------------------------------------------------------
 */
namespace qyh {

// -------------------------------
// 点类型
// -------------------------------
typedef Eigen::Vector3d QyhPoint3D;

// -------------------------------
// 位姿类型（刚体变换）
// -------------------------------
class QyhPose3D {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    // -------------------------------
    // 构造
    // -------------------------------
    QyhPose3D()
        : translation_(Eigen::Vector3d::Zero()),
          rotation_(Eigen::Quaterniond::Identity()) {}

    QyhPose3D(const Eigen::Vector3d &t, const Eigen::Quaterniond &q)
        : translation_(t), rotation_(q) {}

    QyhPose3D(const QyhPoint3D &translation, const QyhPoint3D &rpy_rad) {
        translation_ = translation;
        rotation_ = Eigen::AngleAxisd(rpy_rad[2], Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(rpy_rad[1], Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(rpy_rad[0], Eigen::Vector3d::UnitX());
    }

    QyhPose3D(const std::vector<double> &translation, const std::vector<double> &rpy) {
        QyhPoint3D t(translation[0], translation[1], translation[2]);
        QyhPoint3D r(rpy[0], rpy[1], rpy[2]);
        *this = QyhPose3D(t, r);
    }

    // -------------------------------
    // 工厂函数
    // -------------------------------
    static QyhPose3D FromXYZ(const Eigen::Vector3d &t) {
        return QyhPose3D(t, Eigen::Quaterniond::Identity());
    }

    static QyhPose3D FromRPY(const Eigen::Vector3d &rpy_rad) {
        Eigen::Quaterniond q = Eigen::AngleAxisd(rpy_rad[2], Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(rpy_rad[1], Eigen::Vector3d::UnitY())
                             * Eigen::AngleAxisd(rpy_rad[0], Eigen::Vector3d::UnitX());
        return QyhPose3D(Eigen::Vector3d::Zero(), q);
    }

    static QyhPose3D FromXYZRPY(const Eigen::Vector3d &t, const Eigen::Vector3d &rpy_rad) {
        return QyhPose3D(t, Eigen::AngleAxisd(rpy_rad[2], Eigen::Vector3d::UnitZ())
                             * Eigen::AngleAxisd(rpy_rad[1], Eigen::Vector3d::UnitY())
                             * Eigen::AngleAxisd(rpy_rad[0], Eigen::Vector3d::UnitX()));
    }

    // -------------------------------
    // 逆变换
    // -------------------------------
    QyhPose3D inverse() const {
        Eigen::Quaterniond q_inv = rotation_.conjugate();
        Eigen::Vector3d t_inv = -(q_inv * translation_);
        return QyhPose3D(t_inv, q_inv);
    }

    // -------------------------------
    // 点变换
    // -------------------------------
    QyhPoint3D operator*(const QyhPoint3D &p) const {
        return rotation_ * p + translation_;
    }

    // -------------------------------
    // 位姿变换
    // -------------------------------
    QyhPose3D operator*(const QyhPose3D &other) const {
        Eigen::Vector3d t_new = translation_ + rotation_ * other.translation_;
        Eigen::Quaterniond q_new = rotation_ * other.rotation_;
        return QyhPose3D(t_new, q_new);
    }

    // -------------------------------
    // 打印
    // -------------------------------
    void print(const std::string &name = "T") const {
        Eigen::Vector3d rpy = rotation_.toRotationMatrix().eulerAngles(0,1,2);
        std::cout << name << " translation: " << translation_.transpose() << "\n";
        std::cout << name << " rpy (rad) : " << rpy.transpose() << "\n";
        std::cout << name << " rpy (deg) : " << rpy[0]*180/M_PI << " "
                  << rpy[1]*180/M_PI << " " << rpy[2]*180/M_PI << "\n";
    }

    // -------------------------------
    // 成员访问
    // -------------------------------
    const Eigen::Vector3d &translation() const { return translation_; }
    const Eigen::Quaterniond &rotation() const { return rotation_; }

private:
    Eigen::Vector3d translation_;
    Eigen::Quaterniond rotation_;
};

}  // namespace qyh
