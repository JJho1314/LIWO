#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/se3.hpp"

template<typename T>
inline Eigen::Matrix<T, 4, 4>
interpolateSE3(const Eigen::Matrix<T, 4, 4> &source, const Eigen::Matrix<T, 4, 4> &target, const T alpha) {

//    if(alpha<0 || alpha>1)
//    {
//        std::cerr << "warning: alpha < 0 or alpha > 1" <<std::endl;
//    }

    Sophus::SE3<T> SE1(source);
    Sophus::SE3<T> SE2(target);

    Eigen::Matrix<T, 6, 1> se1 = SE1.log();
    Eigen::Matrix<T, 6, 1> se2 = SE2.log();

    Sophus::SE3<T> SE3t = SE1 * Sophus::SE3<T>::exp(alpha * (se2 - se1));

    return SE3t.matrix();
}

template<typename T>
inline Eigen::Matrix<T, 4, 4> scaleSE3(const Eigen::Matrix<T, 4, 4> &source, const T alpha) {

//    if(alpha<0 || alpha>1)
//    {
//        std::cerr << "warning: alpha < 0 or alpha > 1" <<std::endl;
//    }
    Eigen::Matrix<T, 6, 1> se3 = Sophus::SE3<T>(source).log();
    Sophus::SE3<T> result = Sophus::SE3<T>::exp(alpha * se3);

    return result.matrix();
}