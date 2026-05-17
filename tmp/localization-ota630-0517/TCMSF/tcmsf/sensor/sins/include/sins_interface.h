#pragma once
#include "Eigen/Eigen"
#include "earth.h"
#include <memory>

namespace INS {

struct AVP {
    Eigen::Quaterniond att;
    Eigen::Vector3d    vel;
    Eigen::Vector3d    pos;
    AVP() {
        att = Eigen::Quaterniond::Identity();
        vel << 0.0, 0.0, 0.0;
        pos << 0.0, 0.0, 0.0;
    }
    AVP(const Eigen::Quaterniond &att_, const Eigen::Vector3d &vel_, const Eigen::Vector3d &pos_) {
        att = att_;
        vel = vel_;
        pos = pos_;
    }
    std::vector<double> vector() {
        std::vector<double> v(10);
        v[0] = att.w();
        v[1] = att.x();
        v[2] = att.y();
        v[3] = att.z();
        v[4] = vel.x();
        v[5] = vel.y();
        v[6] = vel.z();
        v[7] = pos.x();
        v[8] = pos.y();
        v[9] = pos.z();
        return v;
    }
};

using Matrix15d = Eigen::Matrix<double, 15, 15>;

class Sins {
public:
    virtual ~Sins(){};

public:
    AVP             avp;
    Eigen::Matrix3d C_b2n;
    Eigen::Matrix3d C_n2b;
    double          ts;
    EARTH           earth;
    Eigen::Vector3d wib;
    Eigen::Vector3d fn;
    Eigen::Vector3d fb;
    Eigen::Vector3d wnb;
    Eigen::Vector3d web;
    Eigen::Vector3d an;
    Eigen::Matrix3d Mpv;
    Eigen::Matrix3d MpvCnb;
    Eigen::Vector3d Mpvvn;
    Eigen::Matrix3d Kg;
    Eigen::Matrix3d Ka;
    Eigen::Vector3d eb;
    Eigen::Vector3d db;
    Eigen::Vector3d tauG;
    Eigen::Vector3d tauA;
    double          tDelay;
    Eigen::Vector3d wm_1;
    Eigen::Vector3d vm_1;

    Matrix15d Ft;

public:
    static std::unique_ptr<Sins> create();
    static std::unique_ptr<Sins> create(const AVP &avp_, double ts_);

public:
    virtual void update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity)                                                                                            = 0;
    virtual void update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity, double ts_, const Eigen::Matrix<double, 6, 1> &ctl_ = Eigen::Matrix<double, 6, 1>::Zero()) = 0;
    virtual void set_state(const AVP &avp_, const Eigen::Vector3d &eb_, const Eigen::Vector3d &db_, bool is_set_att)                                                                              = 0;
};

} // namespace INS