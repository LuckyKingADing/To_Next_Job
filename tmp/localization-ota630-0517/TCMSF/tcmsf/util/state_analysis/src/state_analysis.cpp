#include "state_analysis.h"
#include <cmath>

namespace statistics {
void StateStatistics::insert(const StateInfo &si_) {
    while (stateq.size() >= STATE_SEQ_MAX_SIZE) {
        StateInfo si = stateq.front();
        stateq.pop_front();
        if (statem[si.state] > 0) {
            statem[si.state]--;
        }
    }
    stateq.push_back(si_);
    statem[si_.state]++;
}

void StateStatistics::clear() {
    stateq.clear();
    statem.clear();
}

uint64_t StateStatistics::state_count(uint64_t state) {
    return statem[state];
}

void StateStatistics::set_state_max_num(uint64_t max_) {
    STATE_SEQ_MAX_SIZE = max_;
}

uint64_t StateStatistics::total_state_count() {
    return stateq.size();
}

void StateStatistics::insert_per_sec(const StateInfo &si_) {
    StateInfo si = {std::floor(si_.time), si_.state};
    if (stateq.size() == 0) {
        insert(si);
    } else {
        if (std::abs(stateq.back().time - si.time) > (1.0 - 1e-5)) {
            insert(si);
        }
    }
}

} // namespace statistics