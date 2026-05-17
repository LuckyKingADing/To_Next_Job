#pragma once

#include "fmt/color.h"
#include "fmt/core.h"
#include "fmt/format.h"
#include "modules/localization/src/TCMSF/third_party/RTKLIB-b34k/src/rtklib.h"
#include <fstream>

namespace byd {
namespace tcmsf {

namespace gnss {
class GnssInfo {
public:
    int line_count = 0;

public:
    std::fstream fs;

public:
    bool open(const std::string filepath) {
        fs.open(filepath, std::ios::out);
        if (!fs.is_open()) {
            return false;
        }
    }

public:
    void dump_basic_info(const rtcm_t *const rtcm) {
        fmt::print("MSG Time: {:<16.2f}\n", rtcm->time.time + rtcm->time.sec);
        line_count++;
    }

    void dump_station_info(const rtcm_t *const rtcm) {
        auto sta = rtcm->sta;
        if (rtcm->staid == 0) {
            return;
        }
        fmt::print("{:-<80}", "Station info begin \n");
        line_count++;
        fmt::print("ID: {:<10} Serial Num: {:<10} Type: {:<10}\n", rtcm->staid, sta.antsno, sta.rectype);
        line_count++;
        fmt::print("Position (ecef) (m): {:<10}, {:<10}, {:<10}\n", sta.pos[0], sta.pos[1], sta.pos[2]);
        line_count++;
        fmt::print("{:->80}\n", " station info end");
        line_count++;
    }

    void dump_ephemerides(const rtcm_t *const rtcm) {
        auto nav       = rtcm->nav;
        int  sat_count = 0;
        for (int i = 0; i < nav.n; i++) {
            if (nav.eph[i].sat) {
                sat_count++;
            }
        }
        if (sat_count == 0) {
            return;
        }
        fmt::print("{:─<75}┬{:─<75}\n", "Ephemerides begin ", "");
        line_count++;
        // fmt::print("{:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8}", nav.n, nav.ng, nav.ns, nav.ne, nav.nc, nav.na, nav.nt);
        // fmt::print("{:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8}", nav.utc_gps[0], nav.utc_gps[1], nav.utc_gps[2], nav.utc_gps[3], nav.utc_gps[4], nav.utc_gps[5], nav.utc_gps[6], nav.utc_gps[7]);
        // fmt::print("{:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8}", nav.utc_cmp[0], nav.utc_cmp[1], nav.utc_cmp[2], nav.utc_cmp[3], nav.utc_cmp[4], nav.utc_cmp[5], nav.utc_cmp[6], nav.utc_cmp[7]);
        // fmt::print("{:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8}", nav.ion_gps[0], nav.ion_gps[1], nav.ion_gps[2], nav.ion_gps[3], nav.ion_gps[4], nav.ion_gps[5], nav.ion_gps[6], nav.ion_gps[7]);
        // fmt::print("{:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8} {:<8}", nav.ion_cmp[0], nav.ion_cmp[1], nav.ion_cmp[2], nav.ion_cmp[3], nav.ion_cmp[4], nav.ion_cmp[5], nav.ion_cmp[6], nav.ion_cmp[7]);
        // for (int i = 0; i < MAXSAT; i++) {
        //     fmt::print("{:<8} {:<8} {:<8}", nav.cbias[i][0], nav.cbias[i][1], nav.cbias[i][2]);
        // }
        fmt::print(fmt::emphasis::bold, "{:>3} {:>14} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} │ ", "Sat", "A", "e", "i0", "OMG0", "omg", "M0", "deln", "OMGd", "idot");
        fmt::print(fmt::emphasis::bold, "{:>3} {:>14} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6} {:>6}\n", "Sat", "A", "e", "i0", "OMG0", "omg", "M0", "deln", "OMGd", "idot");
        line_count++;
        int nav_eph_count = 0;
        for (int i = 0; i < nav.n; i++) {
            if (nav.eph[i].sat) {
                nav_eph_count++;
                fmt::print("{:>3} {:>14.3f} {:>6.3f} {:>6.3f} {:>6.3f} {:>6.3f} {:>6.3f} {:>6.3f} {:>6.3f} {:>6.3f} ", nav.eph[i].sat, nav.eph[i].A, nav.eph[i].e, nav.eph[i].i0, nav.eph[i].OMG0, nav.eph[i].omg, nav.eph[i].M0, nav.eph[i].deln, nav.eph[i].OMGd, nav.eph[i].idot);
                if (nav_eph_count % 2 == 0) {
                    fmt::print("\n");
                    line_count++;
                } else {
                    fmt::print("│ ");
                }
            }
        }
        if (nav_eph_count % 2 == 1) {
            fmt::print("\n");
            line_count++;
        }
        fmt::print("{:─>75}┴{:─>75}\n", "", " ephemerides info end");
        line_count++;
    }

    void dump_observation(const rtcm_t *const rtcm) {
        auto obs = rtcm->obs;
        if (obs.n == 0) {
            return;
        }
        fmt::print("{:─<71}┬{:─<66}┬{:─<46}┬{:─<13}\n", "Observation begin ", "", "", "");
        line_count++;
        fmt::print(fmt::emphasis::bold, "{:>3}│r {:^64} │ {:^64} │ {:^44} │ {:^13}\n", "sat", "carrier-phase", "pseudorange", "doppler", "timestamp");
        line_count++;
        for (int i = 0; i < obs.n; i++) {
            switch (satsys(obs.data[i].sat, NULL)) {
                case SYS_GPS:
                    fmt::print(fg(fmt::color::blue) | fmt::emphasis::bold, "{:>3}│{:>1} ", obs.data[i].sat, obs.data[i].rcv);
                    break;
                case SYS_GLO:
                    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold, "{:>3}│{:>1} ", obs.data[i].sat, obs.data[i].rcv);
                    break;
                case SYS_GAL:
                    fmt::print(fg(fmt::color::brown) | fmt::emphasis::bold, "{:>3}│{:>1} ", obs.data[i].sat, obs.data[i].rcv);
                    break;
                case SYS_CMP:
                    fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "{:>3}│{:>1} ", obs.data[i].sat, obs.data[i].rcv);
                    break;
                case SYS_QZS:
                    fmt::print(fg(fmt::color::olive) | fmt::emphasis::bold, "{:>3}│{:>1} ", obs.data[i].sat, obs.data[i].rcv);
                    break;

                default:
                    break;
            }

            fmt::print("{:>12.1f} {:>12.1f} {:>12.1f} {:>12.1f} {:>12.1f} │ {:>12.1f} {:>12.1f} {:>12.1f} {:>12.1f} {:>12.1f} │ {:>8.1f} {:>8.1f} {:>8.1f} {:>8.1f} {:>8.1f} │ {:>12.1f}\n", obs.data[i].L[0], obs.data[i].L[1], obs.data[i].L[2], obs.data[i].L[3], obs.data[i].L[4], obs.data[i].P[0], obs.data[i].P[1], obs.data[i].P[2], obs.data[i].P[3], obs.data[i].P[4], obs.data[i].D[0], obs.data[i].D[1], obs.data[i].D[2], obs.data[i].D[3], obs.data[i].D[4], obs.data[i].time.time + obs.data[i].time.sec);
            line_count++;
        }
        fmt::print("{:─>71}┴{:─>66}┴{:─>60}\n", "", "", " observation info end");
        line_count++;
    }

    void dump_solution(const sol_t *const sol, const std::string msg) {
        fmt::print("{:─<34}┬{:─^39}┬{:─>25}", "Solution begin ", "", "");
        if (msg.length() > 0) {
            for (size_t i = 0; i <= msg.length(); i++)
                fmt::print("{}", '\b');
            fmt::print("{}\n", " " + msg);
        } else {
            fmt::print("{}\n", "");
        }
        line_count++;
        fmt::print(fmt::emphasis::bold, "{:^10}│{:^23}│{:^39}│{:^25}\n", "state", "timestamp (gps/utc)", "position (lla/ecef)", "velocity (enu/ecef)");
        line_count++;
        switch (sol->stat) {
            case SOLQ_NONE:
                fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "{:^10}", "NONE");
                break;
            case SOLQ_FIX:
                fmt::print(fg(fmt::color::green) | fmt::emphasis::bold, "{:^10}", "FIX");
                break;
            case SOLQ_FLOAT:
                fmt::print(fg(fmt::color::blue) | fmt::emphasis::bold, "{:^10}", "FLOAT");
                break;
            case SOLQ_SBAS:
                fmt::print(fg(fmt::color::blue) | fmt::emphasis::bold, "{:^10}", "SBAS");
                break;
            case SOLQ_DGPS:
                fmt::print(fg(fmt::color::blue) | fmt::emphasis::bold, "{:^10}", "DGPS");
                break;
            case SOLQ_SINGLE:
                fmt::print(fg(fmt::color::blue) | fmt::emphasis::bold, "{:^10}", "SINGLE");
                break;
            case SOLQ_PPP:
                fmt::print(fg(fmt::color::green) | fmt::emphasis::bold, "{:^10}", "PPP");
                break;
            case SOLQ_DR:
                fmt::print(fg(fmt::color::red) | fmt::emphasis::bold, "{:^10}", "DR");
                break;

            default:
                break;
        }
        fmt::print("│{:^23.1f}│", sol->time.time + sol->time.sec);
        // double ep[6];
        char t_str[30];
        time2str(sol->time, t_str, 1);
        double pos[3]{0};
        double vel[3]{0};
        if (sol->stat != SOLQ_NONE) {
            if (sol->type == 0) {
                ecef2pos(sol->rr, pos);
                ecef2enu(pos, sol->rr + 3, vel);
            } else if (sol->type == 1) {
                pos[0] = sol->rr[0];
                pos[1] = sol->rr[1];
                pos[2] = sol->rr[2];
                vel[0] = sol->rr[3];
                vel[1] = sol->rr[4];
                vel[2] = sol->rr[5];
            }
        }
        fmt::print("{:>12.6f} {:>12.6f} {:>12.6f} │{:>7.3f} {:>7.3f} {:>7.3f}\n", pos[0] * R2D, pos[1] * R2D, pos[2], vel[0], vel[1], vel[2]);
        line_count++;
        fmt::print("{:^4}/{:^5}│{:^23}│{:>12.2f} {:>12.2f} {:>12.2f} │{:>7.3f} {:>7.3f} {:>7.3f}\n", sol->ns, sol->age, t_str, sol->rr[0], sol->rr[1], sol->rr[2], sol->rr[3], sol->rr[4], sol->rr[5]);
        line_count++;
        fmt::print("{:─^10}┴{:─^23}┴{:─^39}┴{:─>25}\n", "", "", "", " solution end");
        line_count++;
    }
};
} // namespace gnss
} // namespace tcmsf
} // namespace byd