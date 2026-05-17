#pragma once
#include "Eigen/Eigen"
#include "Eigen/Geometry"
#include "highordercoef.h"
#include <memory>

#include <fstream>

#include "rigid_transform.h"
#include "sins_interface.h"

#define ENABLE_IMU_CS_COMPENSATION

// 使用 15 维 SINS模型
// 3 维 姿态
// 3 维 速度
// 3 维 位置
// 3 维 陀螺零偏
// 3 维 加计零偏

namespace INS {

enum CONE_METHOD { CONE_OPTIMAL   = 0,
                   CONE_POLYN     = 1,
                   CONE_UNCOMP    = 2,
                   CONE_HIGHORDER = 3 };

class CS_Compensation {
private:
    Eigen::Matrix<double, 5, 5> cs_coef;

public:
    CS_Compensation();

public:
    void conepolyn(const Eigen::Matrix<double, -1, 3> &wm, Eigen::Vector3d &dphim);
    void coneuncomp(const Eigen::Matrix<double, -1, 3> &wm, Eigen::Vector3d &dphim);
    void conehighorder(const Eigen::Matrix<double, -1, 3> &wm, Eigen::Vector3d &dphim);
    void scullpolyn(const Eigen::Matrix<double, -1, 3> &wm, const Eigen::Matrix<double, -1, 3> &vm, Eigen::Vector3d &scullm);

public:
    void cnscl(const Eigen::Matrix<double, -1, 6> &imu, CONE_METHOD cone_method, Eigen::Vector3d &phim, Eigen::Vector3d &dvbm);
    void cnscl(const Eigen::Matrix<double, 1, 6> &imu, Eigen::Vector3d &wm_, Eigen::Vector3d &vm_, Eigen::Vector3d &phim, Eigen::Vector3d &dvbm);
};

class SinsImpl : public Sins {
public:
    SinsImpl();
    SinsImpl(const AVP &avp_, double ts_);
    ~SinsImpl();
    void init(const AVP &avp_, double ts_);

public:
    virtual void update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity) override final;
    // void update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity, double ts_);
    virtual void update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity, double ts_, const Eigen::Matrix<double, 6, 1> &ctl_ = Eigen::Matrix<double, 6, 1>::Zero()) override final;
    virtual void set_state(const AVP &avp_, const Eigen::Vector3d &eb_, const Eigen::Vector3d &db_, bool is_set_att) override final;

private:
    void update_etm();
};

// using SinsPtr = std::shared_ptr<SinsImpl>;

} // namespace INS