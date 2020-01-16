//
// Created by hyj on 18-11-11.
// Add Notes and completed by HSY on 05-01-2020
//
#include <iostream>
#include <vector>
#include <random>  
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

struct Pose
{
    Pose(Eigen::Matrix3d R, Eigen::Vector3d t):Rwc(R),qwc(R),twc(t) {};
    Eigen::Matrix3d Rwc;
    Eigen::Quaterniond qwc;
    Eigen::Vector3d twc;
};
int main()
{
    int featureNums = 20;
    int poseNums = 10;
    int diem = poseNums * 6 + featureNums * 3;
    double fx = 1.;
    double fy = 1.;
    Eigen::MatrixXd H(diem,diem);
    H.setZero();

    std::vector<Pose> camera_pose;
    double radius = 8;
    for(int n = 0; n < poseNums; ++n ) {
        double theta = n * 2 * M_PI / ( poseNums * 4); // @HSY:  it should be 1/2 PI or 1/4 circle
        // 绕 z轴 旋转
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());
        Eigen::Vector3d t = Eigen::Vector3d(radius * cos(theta) - radius, radius * sin(theta), 1 * sin(2 * theta));
        camera_pose.push_back(Pose(R,t));
    }

    // 随机数生成三维特征点
    std::default_random_engine generator;
    std::vector<Eigen::Vector3d> points;
    for(int j = 0; j < featureNums; ++j)
    {
        std::uniform_real_distribution<double> xy_rand(-4, 4.0);
        std::uniform_real_distribution<double> z_rand(8., 10.);
        double tx = xy_rand(generator);
        double ty = xy_rand(generator);
        double tz = z_rand(generator);

        Eigen::Vector3d Pw(tx, ty, tz);
        points.push_back(Pw);
        // the feature points in the world coordinates

        for (int i = 0; i < poseNums; ++i) {
            Eigen::Matrix3d Rcw = camera_pose[i].Rwc.transpose();
            Eigen::Vector3d Pc = Rcw * (Pw - camera_pose[i].twc);
            // get the camera coordinates of the feature points
            // here, we are assuming all the feature points could be captured by every single camera pose

            double x = Pc.x();
            double y = Pc.y();
            double z = Pc.z();
            double z_2 = z * z;
            Eigen::Matrix<double,2,3> jacobian_uv_Pc;
            //误差关于投影点的导数, 详见slam14讲 P168
            jacobian_uv_Pc<< fx/z, 0 , -x * fx/z_2,
                    0, fy/z, -y * fy/z_2;
            Eigen::Matrix<double,2,3> jacobian_Pj = jacobian_uv_Pc * Rcw;
            Eigen::Matrix<double,2,6> jacobian_Ti;
            jacobian_Ti << -x* y * fx/z_2, (1+ x*x/z_2)*fx, -y/z*fx, fx/z, 0 , -x * fx/z_2,
                            -(1+y*y/z_2)*fy, x*y/z_2 * fy, x/z * fy, 0,fy/z, -y * fy/z_2;
            // 重投影误差关于相机位姿李代数的一阶变化

            H.block(i*6,i*6,6,6) += jacobian_Ti.transpose() * jacobian_Ti;
            H.block(j*3 + 6*poseNums,j*3 + 6*poseNums,3,3) += jacobian_Pj.transpose() * jacobian_Pj;
            H.block(i*6,j*3 + 6*poseNums, 6,3) += jacobian_Ti.transpose() * jacobian_Pj;
            H.block(j*3 + 6*poseNums,i*6 , 3,6) += jacobian_Pj.transpose() * jacobian_Ti;
        }
    }

//    std::cout << H << std::endl;
//    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes(H);
//    std::cout << saes.eigenvalues() <<std::endl;

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    std::cout << svd.singularValues() <<std::endl;

    return 0;
}
