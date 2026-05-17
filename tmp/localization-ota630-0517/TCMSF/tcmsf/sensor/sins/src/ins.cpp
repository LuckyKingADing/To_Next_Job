#include "ins.h"
#include "cyber/common/log.h"
#include <cmath>
#include <iomanip>

namespace INS {

CS_Compensation::CS_Compensation() {
    cs_coef << 0.666666666666667, 0, 0, 0, 0, 0.450000000000000, 1.35000000000000, 0, 0, 0, 0.514285714285714, 0.876190476190476,
        2.03809523809524, 0, 0, 0.496031746031746, 1.04166666666667, 1.28968253968254, 2.72817460317460, 0,
        0.501082251082251, 0.986580086580087, 1.57922077922078, 1.69567099567100, 3.41926406926407;
}

void CS_Compensation::conepolyn(const Eigen::Matrix<double, -1, 3> &wm, Eigen::Vector3d &dphim) {
    switch (wm.rows()) {
        case 1:
            dphim << 0.0, 0.0, 0.0;
            break;
        case 2:
            dphim = 2.0 / 3.0 * wm.row(0).cross(wm.row(1));
            break;
        case 3:
            dphim = 33.0 / 80.0 * wm.row(0).cross(wm.row(2)) +
                    57.0 / 80.0 * wm.row(1).cross(wm.row(2) - wm.row(0));
            break;
        case 4:
            dphim = 736.0 / 945.0 * (wm.row(0).cross(wm.row(1)) + wm.row(2).cross(wm.row(3))) +
                    334.0 / 945.0 * (wm.row(0).cross(wm.row(2)) + wm.row(1).cross(wm.row(3))) +
                    526.0 / 945.0 * (wm.row(0).cross(wm.row(3))) +
                    654.0 / 945.0 * (wm.row(1).cross(wm.row(2)));
            break;
        case 5:
            dphim = 123425.0 / 145152.0 * (wm.row(0).cross(wm.row(1)) + wm.row(3).cross(wm.row(4))) +
                    34875.00 / 145152.0 * (wm.row(0).cross(wm.row(2)) + wm.row(2).cross(wm.row(4))) +
                    90075.00 / 145152.0 * (wm.row(0).cross(wm.row(3)) + wm.row(1).cross(wm.row(4))) +
                    66625.00 / 145152.0 * (wm.row(0).cross(wm.row(4))) +
                    103950.0 / 145152.0 * (wm.row(1).cross(wm.row(2)) + wm.row(2).cross(wm.row(3))) +
                    55400.00 / 145152.0 * (wm.row(1).cross(wm.row(3)));
            break;
        case 6:
            dphim = 9.225974023727258e-01 * (wm.row(0).cross(wm.row(1)) + wm.row(4).cross(wm.row(5))) +
                    8.639610528915165e-02 * (wm.row(0).cross(wm.row(2)) + wm.row(3).cross(wm.row(5))) +
                    7.733225109265687e-01 * (wm.row(0).cross(wm.row(3)) + wm.row(2).cross(wm.row(5))) +
                    3.930627701652648e-01 * (wm.row(0).cross(wm.row(4)) + wm.row(1).cross(wm.row(5))) +
                    5.317640683291427e-01 * (wm.row(0).cross(wm.row(5))) +
                    7.627597403941779e-01 * (wm.row(1).cross(wm.row(2)) + wm.row(3).cross(wm.row(4))) +
                    3.400757575106209e-01 * (wm.row(1).cross(wm.row(3)) + wm.row(2).cross(wm.row(4))) +
                    5.909848488909383e-01 * (wm.row(1).cross(wm.row(4))) +
                    7.071861474024891e-01 * (wm.row(2).cross(wm.row(3)));
            break;
        default:
            dphim << 0.0, 0.0, 0.0;
            break;
    }
}

void CS_Compensation::coneuncomp(const Eigen::Matrix<double, -1, 3> &wm, Eigen::Vector3d &dphim) {
    switch (wm.rows()) {
        case 1:
            dphim << 0.0, 0.0, 0.0;
            break;
        case 2:
            dphim = 2.0 / 3.0 * wm.row(0).cross(wm.row(1));
            break;
        case 3:
            dphim = 27.0 / 40.0 * (wm.row(1).cross(wm.row(2)) + wm.row(0).cross(wm.row(1))) +
                    9.00 / 20.0 * (wm.row(0).cross(wm.row(2)));
            break;
        case 4:
            dphim = 232.0 / 315.0 * (wm.row(2).cross(wm.row(3)) + wm.row(0).cross(wm.row(1))) +
                    46.00 / 105.0 * (wm.row(1).cross(wm.row(3)) + wm.row(0).cross(wm.row(2))) +
                    18.00 / 35.00 * (wm.row(0).cross(wm.row(3))) +
                    178.0 / 315.0 * (wm.row(1).cross(wm.row(2)));
            break;
        case 5:
            dphim = 18575.0 / 24192.0 * wm.row(3).cross(wm.row(4)) +
                    2675.00 / 6048.00 * wm.row(2).cross(wm.row(4)) +
                    11225.0 / 24192.0 * wm.row(1).cross(wm.row(4)) +
                    125.000 / 252.000 * wm.row(0).cross(wm.row(4)) +
                    2575.00 / 6048.00 * wm.row(2).cross(wm.row(3)) +
                    425.000 / 672.000 * wm.row(1).cross(wm.row(3)) +
                    13975.0 / 24192.0 * wm.row(0).cross(wm.row(3)) +
                    1975.00 / 3024.00 * wm.row(1).cross(wm.row(2)) +
                    325.000 / 1512.00 * wm.row(0).cross(wm.row(2)) +
                    21325.0 / 24192.0 * wm.row(0).cross(wm.row(1));
            break;
        default:
            dphim << 0.0, 0.0, 0.0;
            break;
    }
}

void CS_Compensation::conehighorder(const Eigen::Matrix<double, -1, 3> &wm, Eigen::Vector3d &dphim) {
    std::vector<highordercoef::COEF> coefs = highordercoef::hocoef();
    highordercoef::COEF              coef;
    dphim << 0.0, 0.0, 0.0;
    if (wm.rows() >= 2 && wm.rows() <= 5) {
        coef = coefs[wm.rows() - 1];
        for (int i = 0; i < coef.K2.rows(); i++) {
            dphim += coef.K2(i) * wm.row(coef.ij(i, 0)).cross(wm.row(coef.ij(i, 1)));
        }
        for (int i = 0; i < coef.K3.rows(); i++) {
            dphim += coef.K3(i) * wm.row(coef.ijk(i, 0)).cross(wm.row(coef.ijk(i, 1)).cross(wm.row(coef.ijk(i, 2))));
        }
        for (int i = 0; i < coef.K4.rows(); i++) {
            dphim += coef.K4(i) * wm.row(coef.ijkl(i, 0)).cross(wm.row(coef.ijkl(i, 1)).cross(wm.row(coef.ijkl(i, 2)).cross(wm.row(coef.ijkl(i, 3)))));
        }
    }
}

void CS_Compensation::scullpolyn(const Eigen::Matrix<double, -1, 3> &wm, const Eigen::Matrix<double, -1, 3> &vm, Eigen::Vector3d &scullm) {
    Eigen::Index N = wm.rows();
    switch (N) {
        case 1:
            scullm << 0.0, 0.0, 0.0;
            break;
        case 2:
            scullm = 2.0 / 3.0 * (wm.row(0).cross(vm.row(1)) + wm.row(0).cross(vm.row(1)));
            break;
        case 3:
            scullm = 33.0 / 80.0 * (wm.row(0).cross(vm.row(2)) + vm.row(0).cross(wm.row(2))) +
                     57.0 / 80.0 * (wm.row(0).cross(vm.row(1)) + wm.row(1).cross(vm.row(2)) + vm.row(0).cross(wm.row(1)) + vm.row(1).cross(wm.row(2)));
            break;
        case 4:
            scullm = 736.0 / 945.0 * (wm.row(0).cross(vm.row(1)) + wm.row(2).cross(vm.row(3)) + vm.row(0).cross(wm.row(1)) + vm.row(2).cross(wm.row(3))) +
                     334.0 / 945.0 * (wm.row(0).cross(vm.row(2)) + wm.row(1).cross(vm.row(3)) + vm.row(0).cross(wm.row(2)) + vm.row(1).cross(wm.row(3))) +
                     526.0 / 945.0 * (wm.row(0).cross(vm.row(3)) + vm.row(0).cross(wm.row(3))) +
                     654.0 / 945.0 * (wm.row(1).cross(vm.row(2)) + vm.row(1).cross(wm.row(2)));
            break;

        default:
            scullm << 0.0, 0.0, 0.0;
            break;
    }
}

void CS_Compensation::cnscl(const Eigen::Matrix<double, -1, 6> &imu, CONE_METHOD cone_method, Eigen::Vector3d &phim, Eigen::Vector3d &dvbm) {

    Eigen::Index N = imu.rows();

    Eigen::Vector3d dphim(0.0, 0.0, 0.0);
    Eigen::Vector3d wmm(0.0, 0.0, 0.0);

    Eigen::Matrix<double, -1, 3> wm = imu.block(0, 0, N, 3);

    wmm = wm.colwise().sum();

    Eigen::Vector3d cm(0.0, 0.0, 0.0);

    switch (cone_method) {
        case CONE_METHOD::CONE_OPTIMAL: {
            Eigen::Matrix<double, 1, -1> c_ = cs_coef.block(N - 2, 0, 1, N - 1);
            Eigen::MatrixXd              w_ = wm.block(0, 0, N - 1, wm.cols());

            cm    = c_ * w_;
            dphim = cm.cross(wm.row(N - 1));
        } break;

        case CONE_METHOD::CONE_POLYN:
            conepolyn(wm, dphim);
            break;

        case CONE_METHOD::CONE_UNCOMP:
            coneuncomp(wm, dphim);
            break;

        case CONE_METHOD::CONE_HIGHORDER:
            conehighorder(wm, dphim);
            break;
        default:
            break;
    }

    phim = wmm + dphim;

    Eigen::Matrix<double, -1, 3> vm = imu.block(0, 3, N, 3);
    Eigen::Vector3d              vmm(0.0, 0.0, 0.0);
    Eigen::Vector3d              scullm(0.0, 0.0, 0.0);

    vmm = vm.colwise().sum();

    switch (cone_method) {
        case CONE_METHOD::CONE_OPTIMAL: {
            Eigen::Matrix<double, 1, -1> c_ = cs_coef.block(N - 2, 0, 1, N - 1);
            Eigen::MatrixXd              v_ = vm.block(0, 0, N - 1, vm.cols());
            Eigen::Vector3d              sm = c_ * v_;
            scullm                          = cm.cross(vm.row(N - 1)) + sm.cross(wm.row(N - 1));
        } break;

        default:
            scullpolyn(wm, vm, scullm);
            break;
    }

    Eigen::Vector3d rotm = 1.0 / 2.0 * (wmm.cross(vmm));
    dvbm                 = vmm + rotm + scullm;
}

void CS_Compensation::cnscl(const Eigen::Matrix<double, 1, 6> &imu, Eigen::Vector3d &wm_, Eigen::Vector3d &vm_, Eigen::Vector3d &phim, Eigen::Vector3d &dvbm) {

    Eigen::Vector3d wm = imu.block<1, 3>(0, 0);
    Eigen::Vector3d vm = imu.block<1, 3>(0, 3);

    Eigen::Vector3d dphim  = 1.0 / 12.0 * (wm_.cross(wm));
    Eigen::Vector3d scullm = 1.0 / 12.0 * (wm_.cross(vm) + vm_.cross(wm));
    Eigen::Vector3d rotm   = 1.0 / 2.00 * (wm.cross(vm));

    phim = wm + dphim;
    dvbm = vm + rotm + scullm;

    wm_ = wm;
    vm_ = vm;
}

SinsImpl::SinsImpl() {
    AVP    avp_;
    double ts_ = 0.01;
    init(avp_, ts_);
}

SinsImpl::SinsImpl(const AVP &avp_, double ts_) { init(avp_, ts_); }

SinsImpl::~SinsImpl() { AINFO << "SINS_IMPL exit"; }

void SinsImpl::init(const AVP &avp_, double ts_) {
    avp = avp_;
    ts  = ts_;
    earth.update(avp.pos, avp.vel);
    C_b2n = avp.att.toRotationMatrix();
    C_n2b = C_b2n.transpose();
    wib   = C_n2b * earth.wnin;
    fn    = -earth.g;
    fb    = C_n2b * fn;
    wnb << 0.0, 0.0, 0.0;
    web << 0.0, 0.0, 0.0;
    an << 0.0, 0.0, 0.0;
    Mpv << 0.0, 1.0 / earth.RMh, 0.0, 1.0 / earth.clRNh, 0.0, 0.0, 0.0, 0.0, 1.0;
    MpvCnb = Mpv * C_b2n;
    Mpvvn  = Mpv * avp.vel;
    Kg     = Eigen::Matrix3d::Identity();
    Ka     = Eigen::Matrix3d::Identity();
    eb     = Eigen::Vector3d::Zero();
    db     = Eigen::Vector3d::Zero();
    tauG << HUGE_VAL_F64, HUGE_VAL_F64, HUGE_VAL_F64;
    tauA << HUGE_VAL_F64, HUGE_VAL_F64, HUGE_VAL_F64;
    tDelay = 0.0;
    wm_1 << 0.0, 0.0, 0.0;
    vm_1 << 0.0, 0.0, 0.0;
    Ft = Matrix15d::Zero();
    update_etm();

    // fs.open("modules/localization/src/TCMSF/test/tcmsf_log/ins.csv", std::ios::out);
}

void SinsImpl::update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity) {

    CS_Compensation cs;

    size_t N   = imu.rows();
    double nts = N * ts;

    Eigen::Vector3d phim, dvbm;

#ifdef ENABLE_IMU_CS_COMPENSATION

    if (N == 2 || N == 3 || N == 4) {
        cs.cnscl(imu, CONE_METHOD::CONE_HIGHORDER, phim, dvbm);
    } else if (N == 1) {
        cs.cnscl(imu, wm_1, vm_1, phim, dvbm);
    }

#else

    {
        Eigen::Matrix<double, -1, 3> wm  = imu.block(0, 0, N, 3);
        Eigen::Matrix<double, -1, 3> vm  = imu.block(0, 3, N, 3);
        Eigen::Vector3d              wmm = wm.colwise().sum();
        Eigen::Vector3d              vmm = vm.colwise().sum();

        phim = wmm;
        dvbm = vmm;
    }

#endif

    phim = phim + phi_b_gravity;

    phim = Kg * phim - eb * nts;
    dvbm = Ka * dvbm - db * nts;
    //
    Eigen::Vector3d vel01 = avp.vel + an * nts / 2.0;
    Eigen::Vector3d pos01 = avp.pos + Mpv * vel01 * nts / 2.0;
    earth.update(pos01, vel01);
    wib   = phim / nts;
    fb    = dvbm / nts;
    C_b2n = avp.att.toRotationMatrix();
    C_n2b = C_b2n.transpose();
    web   = wib - C_n2b * earth.wnie;
    wnb   = wib - (C_b2n * rv2m(phim / 2.0)).transpose() * earth.wnin;

    MpvCnb = Mpv * C_b2n;

    //
    fn                   = C_b2n * fb;
    an                   = rv2m(-earth.wnin * nts / 2.0) * fn + earth.g_cc;
    Eigen::Vector3d vel1 = avp.vel + an * nts;

    //
    Mpv(0, 1) = 1.0 / earth.RMh;
    Mpv(1, 0) = 1.0 / earth.clRNh;
    Mpvvn     = Mpv * (avp.vel + vel1) / 2.0;
    avp.pos   = avp.pos + Mpvvn * nts;
    avp.vel   = vel1;

    //
    avp.att = rv2q(-earth.wnin * nts) * avp.att * rv2q(phim);

    avp.att.normalize();

    // //
    // avpL.att = avp.att;
    // avpL.vel = avp.vel + C_b2n * Skew(web) * lever;
    // avpL.pos = avp.pos + MpvCnb * lever;

    // fs << std::setprecision(14) << avp.vel.x() << "," << avp.vel.y() << "," << avp.vel.z() << "," << avp.pos.x() * 180 / M_PI << "," << avp.pos.y() * 180 / M_PI << "," << avp.pos.z() << "\n";

    update_etm();
}

// void SinsImpl::update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity, double ts_) {
//     ts = ts_;
//     update(imu, phi_b_gravity);
// }

void SinsImpl::update(const Eigen::Matrix<double, -1, 6> &imu, const Eigen::Vector3d &phi_b_gravity, double ts_, const Eigen::Matrix<double, 6, 1> &ctl_) {
    ts = ts_;
    update(imu, phi_b_gravity);
    size_t N = imu.rows();

    Eigen::Vector3d angle_vel_ctl = ctl_.block<3, 1>(0, 0);
    Eigen::Vector3d vel_ctl       = ctl_.block<3, 1>(3, 0);
    avp.pos                       = avp.pos + N * vel_ctl;
    avp.att                       = avp.att * rv2q(angle_vel_ctl * N);
}

void SinsImpl::update_etm() {
    double tl      = earth.tl;
    double secl    = 1.0 / earth.cl;
    double f_RMh   = 1.0 / earth.RMh;
    double f_RNh   = 1.0 / earth.RNh;
    double f_clRNh = 1.0 / earth.clRNh;
    double f_RMh2  = f_RMh * f_RMh;
    double f_RNh2  = f_RNh * f_RNh;

    Eigen::Vector3d vn       = avp.vel;
    double          vE_clRNh = vn.x() * f_clRNh;
    double          vE_RNh2  = vn.x() * f_RNh2;
    double          vN_RMh2  = vn.y() * f_RMh2;

    Eigen::Matrix3d Mp1 = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d Mp2 = Eigen::Matrix3d::Zero();
    Mp1.block<2, 1>(1, 0) << -earth.wnie.z(), earth.wnie.y();
    Mp2.block<1, 1>(2, 0) << vE_clRNh * secl;
    Mp2.block<3, 1>(0, 2) << vN_RMh2, -vE_RNh2, -vE_RNh2 * tl;

    Eigen::Matrix3d Avn = Skew(vn);
    Eigen::Matrix3d Awn = Skew(earth.wnien);

    Eigen::Matrix3d Maa = -Skew(earth.wnin);

    Eigen::Matrix3d Mav = Eigen::Matrix3d::Zero();
    Mav.block<2, 1>(1, 0) << f_RNh, f_RNh * tl;
    Mav.block<1, 1>(0, 1) << -f_RMh;

    Eigen::Matrix3d Map = Mp1 + Mp2;

    Eigen::Matrix3d Mva = Skew(fn);

    Eigen::Matrix3d Mvv = Avn * Mav - Awn;
    Eigen::Matrix3d Mvp = Avn * (Mp1 + Map);

    double scl = earth.sl * earth.cl;
    Mvp(2, 0)  = Mvp(2, 0) - glv.g0 * (5.2790414e-3 * 2 + 2.32718e-5 * 4 * earth.sl2) * scl;
    Mvp(2, 2)  = Mvp(2, 2) + 3.086e-6;

    Eigen::Matrix3d Mpp = Eigen::Matrix3d::Zero();
    Mpp.block<1, 1>(1, 0) << vE_clRNh * tl;
    Mpp.block<2, 1>(0, 2) << -vN_RMh2, -vE_RNh2 * secl;

    Ft.setZero();
    Ft.block<3, 3>(0, 0) = Maa;
    Ft.block<3, 3>(0, 3) = Mav;
    Ft.block<3, 3>(0, 6) = Map;
    Ft.block<3, 3>(0, 9) = -C_b2n;

    Ft.block<3, 3>(3, 0)  = Mva;
    Ft.block<3, 3>(3, 3)  = Mvv;
    Ft.block<3, 3>(3, 6)  = Mvp;
    Ft.block<3, 3>(3, 12) = C_b2n;

    Ft.block<3, 3>(6, 3) = Mpv;
    Ft.block<3, 3>(6, 6) = Mpp;

    Eigen::Matrix3d MtauG  = (-1.0 / tauG.array()).matrix().asDiagonal();
    Eigen::Matrix3d MtauA  = (-1.0 / tauA.array()).matrix().asDiagonal();
    Ft.block<3, 3>(9, 9)   = MtauG;
    Ft.block<3, 3>(12, 12) = MtauA;
}

void SinsImpl::set_state(const AVP &avp_, const Eigen::Vector3d &eb_, const Eigen::Vector3d &db_, bool is_set_att) {
    if (is_set_att) {
        avp.att = avp_.att;
    }
    avp.pos = avp_.pos;
    avp.vel = avp_.vel;
    eb      = eb_;
    db      = db_;
}

} // namespace INS