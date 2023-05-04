#include <ahrs/Propagator.h>

#include <ahrs/Utils.h>

namespace OriEst {

    Propagator::Propagator(const double &gyro_noise, const double &gyro_bias_noise)
            : gyro_noise_(gyro_noise), gyro_bias_noise_(gyro_bias_noise) {}

    void Propagator::PropagateMean(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro,
                                   const double delta_t, Eigen::Matrix3d &end_G_R_I) {
        // Mean propagation.
        const Eigen::Vector3d unbiased_gyro = gyro - Eigen::Vector3d::Zero();
        const Eigen::Vector3d angle_vec = unbiased_gyro * delta_t;
        Eigen::Matrix3d delta_rot;
        if (angle_vec.norm() < 1e-12) {
            delta_rot = Eigen::Quaterniond(
                    Eigen::Matrix3d::Identity() + SkewMat(angle_vec)).normalized().toRotationMatrix();
        } else {
            const double angle = angle_vec.norm();
            const Eigen::Vector3d axis = angle_vec / angle;
            delta_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }
        end_G_R_I = begin_G_R_I * delta_rot;
    }

    void Propagator::EstimateByAcc(const Eigen::Vector3d &acc, Eigen::Matrix3d &end_G_R_I) {

        double roll = atan2(acc(1), acc(2));
        double pitch = -atan2(acc(0), sqrt(pow(acc(1), 2) + pow(acc(2), 2)));
        double yaw = 0.0;
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()));
        end_G_R_I = yawAngle * pitchAngle * rollAngle;

    }

//龙格库塔
    void Propagator::PropagateRK(const Eigen::Matrix3d &begin_G_R_I, const Eigen::Vector3d &gyro,
                                 const Eigen::Vector3d &last_gyro, const double delta_t, Eigen::Matrix3d &end_G_R_I) {
        Eigen::Quaterniond quaternion_self_last(begin_G_R_I);
        double dif_quarterion_f[4];
        double dif_quarterion_l[4];
        double med_quarterion[4];

        dif_quarterion_f[0] = (-quaternion_self_last.x() * last_gyro[0] - quaternion_self_last.y() * last_gyro[1] -
                               quaternion_self_last.z() * last_gyro[2]) * 0.5f;
        dif_quarterion_f[1] = (quaternion_self_last.w() * last_gyro[0] + quaternion_self_last.y() * last_gyro[2] -
                               quaternion_self_last.z() * last_gyro[1]) * 0.5f;
        dif_quarterion_f[2] = (quaternion_self_last.w() * last_gyro[1] - quaternion_self_last.x() * last_gyro[2] +
                               quaternion_self_last.z() * last_gyro[0]) * 0.5f;
        dif_quarterion_f[3] = (quaternion_self_last.w() * last_gyro[2] + quaternion_self_last.x() * last_gyro[1] -
                               quaternion_self_last.y() * last_gyro[0]) * 0.5f;

        med_quarterion[0] = quaternion_self_last.w() + dif_quarterion_f[0] * delta_t;
        med_quarterion[1] = quaternion_self_last.x() + dif_quarterion_f[1] * delta_t;
        med_quarterion[2] = quaternion_self_last.y() + dif_quarterion_f[2] * delta_t;
        med_quarterion[3] = quaternion_self_last.z() + dif_quarterion_f[3] * delta_t;

        dif_quarterion_l[0] =
                (-med_quarterion[1] * gyro[0] - med_quarterion[2] * gyro[1] - med_quarterion[3] * gyro[2]) * 0.5f;
        dif_quarterion_l[1] =
                (med_quarterion[0] * gyro[0] + med_quarterion[2] * gyro[2] - med_quarterion[3] * gyro[1]) * 0.5f;
        dif_quarterion_l[2] =
                (med_quarterion[0] * gyro[1] - med_quarterion[1] * gyro[2] + med_quarterion[3] * gyro[0]) * 0.5f;
        dif_quarterion_l[3] =
                (med_quarterion[0] * gyro[2] + med_quarterion[1] * gyro[1] - med_quarterion[2] * gyro[0]) * 0.5f;

        Eigen::Quaterniond quaternion;
        quaternion.w() = quaternion_self_last.w() + 0.5f * (dif_quarterion_f[0] + dif_quarterion_l[0]) * delta_t;
        quaternion.x() = quaternion_self_last.x() + 0.5f * (dif_quarterion_f[1] + dif_quarterion_l[1]) * delta_t;
        quaternion.y() = quaternion_self_last.y() + 0.5f * (dif_quarterion_f[2] + dif_quarterion_l[2]) * delta_t;
        quaternion.z() = quaternion_self_last.z() + 0.5f * (dif_quarterion_f[3] + dif_quarterion_l[3]) * delta_t;
        end_G_R_I = quaternion.matrix();
    }

    void Propagator::PropagateMeanAndCov(const Eigen::Matrix3d &begin_G_R_I,
                                         const Eigen::Vector3d &begin_bg,
                                         const Eigen::Matrix<double, 6, 6> &begin_cov,
                                         const Eigen::Vector3d &gyro,
                                         const double delta_t,
                                         Eigen::Matrix3d *end_G_R_I,
                                         Eigen::Vector3d *end_bg,
                                         Eigen::Matrix<double, 6, 6> *end_cov) {
        // Mean propagation.
        const Eigen::Vector3d unbiased_gyro = gyro - begin_bg;
        const Eigen::Vector3d angle_vec = unbiased_gyro * delta_t;
        Eigen::Matrix3d delta_rot;
        if (angle_vec.norm() < 1e-12) {
            delta_rot = Eigen::Quaterniond(
                    Eigen::Matrix3d::Identity() + SkewMat(angle_vec)).normalized().toRotationMatrix();
        } else {
            const double angle = angle_vec.norm();
            const Eigen::Vector3d axis = angle_vec / angle;
            delta_rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
        }
        *end_G_R_I = begin_G_R_I * delta_rot;
        *end_bg = begin_bg;

        // Jacobian.
        Eigen::Matrix<double, 6, 6> Fx;
        Fx.topLeftCorner<3, 3>() = delta_rot.transpose();
        Fx.topRightCorner<3, 3>() = -Eigen::Matrix3d::Identity() * delta_t;
        Fx.bottomLeftCorner<3, 3>() = Eigen::Matrix3d::Zero();
        Fx.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        Q.topLeftCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_noise_ * delta_t * delta_t;
        Q.bottomRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * gyro_bias_noise_ * delta_t;

        *end_cov = Fx * begin_cov * Fx.transpose() + Q;
    }

}  // namespac OriEst