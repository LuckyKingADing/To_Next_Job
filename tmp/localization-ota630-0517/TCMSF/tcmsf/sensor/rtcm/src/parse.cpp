#include "parse.h"

namespace byd {
namespace tcmsf {
namespace rtcm {

/* carrier-phase bias (fcb) correction ---------------------------------------*/
static void corr_phase_bias(obsd_t *obs, int n, const nav_t *nav) {
    double  freq;
    uint8_t code;
    int     i, j;

    for (i = 0; i < n; i++)
        for (j = 0; j < NFREQ; j++) {
            code = obs[i].code[j];
            if ((freq = sat2freq(obs[i].sat, code, nav)) == 0.0)
                continue;

            /* correct phase bias (cyc) */
            obs[i].L[j] -= nav->ssr[obs[i].sat - 1].pbias[code - 1] * freq / CLIGHT;
        }
}

ResolveImpl::ResolveImpl() :
    shall_exit(false) {
    opt = prcopt_default;
    {
        opt.mode       = PMODE_KINEMA;
        opt.soltype    = 0;
        opt.nf         = 1;
        opt.modear     = 3;
        opt.glomodear  = 1;
        opt.arfilter   = 1;
        opt.minlock    = 0; // 0 5
        opt.minfix     = 20;
        opt.maxout     = 10;
        opt.thresar[0] = 3.0;
        opt.thresar[1] = 0.02; //   {3.0, 0.25, 0.0, 1E-9, 1E-5, 3.0, 3.0, 0.0};
        opt.navsys     = SYS_CMP | SYS_GPS | SYS_GAL | SYS_GLO;
        opt.elmin      = 15.0 * D2R;
        opt.sateph     = EPHOPT_BRDC;
        opt.bdsmodear  = 1;
        opt.dynamics   = 2;
        opt.ionoopt    = IONOOPT_OFF;
        opt.tropopt    = TROPOPT_OFF;
        opt.refpos     = 4;
        opt.outsingle  = 0;
        opt.niter      = 1;

        opt.snrmask.ena[0] = 1;
        opt.snrmask.ena[1] = 1;
        for (int i = 0; i < 5; i++) {
            opt.snrmask.mask[i][0] = 46; // 45; // 5
            opt.snrmask.mask[i][1] = 46; // 45; // 15
            opt.snrmask.mask[i][2] = 45; // 42; // 25
            opt.snrmask.mask[i][3] = 44; // 40; // 35
            opt.snrmask.mask[i][4] = 40; // 37; // 45
            opt.snrmask.mask[i][5] = 35; // 35; // 55
            opt.snrmask.mask[i][6] = 35; // 33; // 65
            opt.snrmask.mask[i][7] = 35; // 42; // 75
            opt.snrmask.mask[i][8] = 35; // 42; // 85
        }
    }

    rtkinit(&rtk, &opt);
    obs.data        = (obsd_t *)calloc(MAXOBS * 2, sizeof(obsd_t));
    rover_msg_queue = BlockingConcurrentQueue<uint8_t>(RTCM_MSG_QUEUE_LENGTH);
    base_msg_queue  = BlockingConcurrentQueue<uint8_t>(RTCM_MSG_QUEUE_LENGTH);
    if (!init_rtcm(&rover_rtcm)) {
        AERROR << "rover rtcm init failed!";
        throw;
    }
    if (!init_rtcm(&base_rtcm)) {
        AERROR << "base rtcm init failed!";
        throw;
    }

    start_daemon();
}

void ResolveImpl::register_solve_cb(std::function<void(void)> cb_) {
    change_cb.lock();
    solve_cb = cb_;
    change_cb.unlock();
}

ResolveImpl::ResolveImpl(double timepoint[6]) {
    ResolveImpl();
    base_rtcm.time  = utc2gpst(epoch2time(timepoint));
    rover_rtcm.time = utc2gpst(epoch2time(timepoint));
}

ResolveImpl::~ResolveImpl() {
    shall_exit.store(true);
    if (parse_base_t.joinable()) {
        parse_base_t.join();
    }
    if (parse_rover_t.joinable()) {
        parse_rover_t.join();
    }
    rtkfree(&rtk);
    free_rtcm(&rover_rtcm);
    free_rtcm(&base_rtcm);
    freeobs(&obs);
    AINFO << "RTCM ResolveImpl exit";
}

void ResolveImpl::write_base(uint8_t byte) {
    base_msg_byte_count++;
    moodycamel::ProducerToken base_ptok(base_msg_queue);
    base_msg_queue.enqueue(base_ptok, byte);
    while (base_msg_queue.size_approx() > RTCM_MSG_QUEUE_LENGTH) {
        uint8_t tmp;
        base_msg_queue.try_dequeue(tmp);
        // avoide log flood, [ the size of per msg pack is about 2kB ]
        if ((base_msg_byte_count - base_msg_queue_overflow_idx) > 2000)
            AWARN << "base msg queue too large, dequeue";
        base_msg_queue_overflow_idx = base_msg_byte_count;
    }
}
void ResolveImpl::write_rover(uint8_t byte) {
    rover_msg_byte_count++;
    moodycamel::ProducerToken rover_ptok(rover_msg_queue);
    rover_msg_queue.enqueue(rover_ptok, byte);
    while (rover_msg_queue.size_approx() > RTCM_MSG_QUEUE_LENGTH) {
        uint8_t tmp;
        base_msg_queue.try_dequeue(tmp);
        // avoide log flood
        if ((rover_msg_byte_count - rover_msg_queue_overflow_idx) > 2000)
            AWARN << "rover msg queue too large, dequeue";
        rover_msg_queue_overflow_idx = rover_msg_byte_count;
    }
}

void ResolveImpl::parse_base() {
    uint8_t byte = 0;
    while (true) {
        if (base_msg_queue.wait_dequeue_timed(byte, std::chrono::milliseconds(400))) {
            int ret = input_rtcm3(&base_rtcm, byte);
            if (ret == 1) {
                AINFO << "get base obs msg";
                if (true) {
                    gnss_info.dump_observation(&base_rtcm);
                }
            }
        }
        if (shall_exit.load()) {
            AINFO << "parse_base thread exit";
            break;
        }
    }
}
void ResolveImpl::parse_rover() {
    uint8_t byte = 0;
    while (true) {
        if (rover_msg_queue.wait_dequeue_timed(byte, std::chrono::milliseconds(400))) {
            int ret = input_rtcm3(&rover_rtcm, byte);
            if (ret == 1) {
                AINFO << "get rover obs msg, do resolve";
                resolve();
                if (true) {
                    gnss_info.dump_observation(&rover_rtcm);
                    // gnss_info.dump_solution(&rtk.sol, "\0");
                }
                change_cb.lock();
                if (solve_cb) {
                    solve_cb();
                }
                change_cb.unlock();
            }
        }
        if (shall_exit.load()) {
            AINFO << "parse_rover thread exit";
            break;
        }
    }
}

bool ResolveImpl::resolve() {
    // rtk pos

    obs.n = 0;
    for (int i = 0; i < rover_rtcm.obs.n && obs.n < MAXOBS * 2; i++) {
        obs.data[obs.n++]       = rover_rtcm.obs.data[i];
        obs.data[obs.n - 1].rcv = 1;
    }

    base_rtcm_mutex.lock();
    for (int i = 0; i < base_rtcm.obs.n && obs.n < MAXOBS * 2; i++) {
        obs.data[obs.n++]       = base_rtcm.obs.data[i];
        obs.data[obs.n - 1].rcv = 2;
    }
    base_rtcm_mutex.unlock();

    /* carrier phase bias correction */
    if (!strstr(rtk.opt.pppopt, "-DIS_FCB")) {
        corr_phase_bias(obs.data, obs.n, &rover_rtcm.nav);
    }
    rtk.opt.rb[0] = base_rtcm.sta.pos[0];
    rtk.opt.rb[1] = base_rtcm.sta.pos[1];
    rtk.opt.rb[2] = base_rtcm.sta.pos[2];
    /* rtk positioning */
    return rtkpos(&rtk, obs.data, obs.n, &rover_rtcm.nav);
}

void ResolveImpl::start_daemon() {
    parse_base_t  = std::thread([this]() {
        AINFO << "start parse_base thread";
        std::string name = "parse_base";
        pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
        parse_base();
    });
    parse_rover_t = std::thread([this]() {
        AINFO << "start parse_rover thread";
        std::string name = "parse_rover";
        pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
        parse_rover();
    });
}

} // namespace rtcm
} // namespace tcmsf
} // namespace byd