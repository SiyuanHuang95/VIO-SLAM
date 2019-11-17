#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <sophus/so3.h>

int main() {
  // We set the rotation to be 45 degree, alongside the z-axis
  Eigen::Matrix3d current_R =
      Eigen::AngleAxisd(M_PI / 4, Eigen::Vector3d(0.0, 0.0, 1.0))
          .toRotationMatrix();
  std::cout << " the Current R: \n" << current_R << std::endl;

  Sophus::SO3 current_SO3_R(current_R);
  std::cout << " the current R with represent of SO3: \n"
            << current_SO3_R << std::endl;

  Eigen::Quaterniond current_q(current_R);
  std::cout << " the current R with represent of Quaterniond \n"
            << current_q.coeffs().transpose() << std ::endl;

  // set the update value
  Eigen::Vector3d update_w = Eigen ::Vector3d(0.01, 0.02, 0.03);
  std::cout << "used update vector: \n" << update_w.transpose() << std::endl;

  // Type One: Update with Rotation Matrix
  // R <- R * exp(w^)
  Sophus::SO3 update_R_rotationMatrix =
      current_SO3_R * Sophus::SO3::exp(update_w);
  std::cout << " the update with Rotation Matrix: \n"
            << update_R_rotationMatrix.matrix() << std::endl;

  // Type Two: Update with Quaterniond
  // q <- q cross dot [1, 1/2 w]^T
  Eigen::Quaterniond update_q;
  update_q.w() = 1;
  update_q.x() = 1 / 2 * update_w.x();
  update_q.y() = 1 / 2 * update_w.y();
  update_q.z() = 1 / 2 * update_w.z();

  update_q = update_q.normalized();
  // Remember to normalize the quaterion, cuz only unit quaterion could
  // represent the rotation
  Eigen::Quaterniond update_q_from_Quaternion = current_q * update_q;
  std::cout << " the update with Quaterion: \n"
            << update_q_from_Quaternion.normalized().toRotationMatrix()
            << std::endl;
}
