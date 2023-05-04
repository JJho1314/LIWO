#pragma once

#include <Eigen/Dense>

namespace OriEst {

    class Propagator {
    public:
        Propagator(const double &gyro_noise, const double &gyro_bias_noise);

        void PropagateMeanAndCov(const Eigen::Matrix3d &begin_G_R_I,
                                 const Eigen::Vector3d &begin_bg,
                                 const Eigen::Matrix<double, 6, 6> &begin_cov,
                                 const Eigen::Vector3d &gyro,
                                 const double delta_t,
                                 Eigen::Matrix3d *end_G_R_I,
                                 Eigen::Vector3d *end_bg,
                                 Eigen::Matrix<double, 6, 6> *end_cov);

        void PropagateMean(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro,
                           const double delta_t, Eigen::Matrix3d &end_G_R_I);

        void PropagateRK(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro,
                         const Eigen::Vector3d &last_gyro, const double delta_t, Eigen::Matrix3d &end_G_R_I);

        void EstimateByAcc(const Eigen::Vector3d &acc, Eigen::Matrix3d &end_G_R_I);

    private:
        double gyro_noise_;
        double gyro_bias_noise_;
    };

}  // namespace OriEst