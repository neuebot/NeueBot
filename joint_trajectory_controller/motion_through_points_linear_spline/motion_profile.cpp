#include "motion_profile.hpp"
#include <cmath>
#include <limits>

using namespace std;

template <typename T> int sgn(T val) {
    return (T(0) <= val) - (val < T(0));
}

//TODO: This MotionProfile considers initial and end velocities/acceleration to be 0.
MotionProfile::MotionProfile(const vector<double> &via_points, const vector<double> &time_via_points,
                             const double limit_acceleration, const double inner_acceleration) :
    N(via_points.size()),
    m_q(via_points),
    m_t(time_via_points),
    m_a_lim(limit_acceleration),
    m_a_in(inner_acceleration)
{
    m_dt.resize(N-1);
    m_bt.resize(N);
    m_ba.resize(N);
    m_lt.resize(N-1);
    m_lv.resize(N-1);

    m_dt[0] = m_t[1] - m_t[0];
    for(size_t i = 1; i<N-1; ++i) {
        m_dt[i] = m_t[i+1] - m_t[i];
        m_lv[i] = (m_q[i+1]-m_q[i]) / m_dt[i];
    }
}

void MotionProfile::CalculateSegmentTimes(void) {
    //First Segment
    m_ba[0] = m_a_lim * sgn(m_q[1] - m_q[0]);
    m_bt[0] = m_dt[0] - sqrt(pow(m_dt[0], 2) - (2 * (m_q[1]-m_q[0]) / m_ba[0]));
    m_lv[0] = (m_q[1] - m_q[0]) / (m_dt[0] - (0.5 * m_bt[0]));

    //Last Segment
    m_ba[N-1] = m_a_lim * sgn(m_q[N-2] - m_q[N-1]);
    m_bt[N-1] = m_dt[N-2] - sqrt(pow(m_dt[N-2],2) + (2 * (m_q[N-1] - m_q[N-2]) / m_ba[N-1]));
    m_lv[N-2] = (m_q[N-1] - m_q[N-2]) / (m_dt[N-2] - (0.5 * m_bt[N-1]));

    //Inner Segments
    for(size_t i=1; i<N-1; ++i) {
        m_ba[i] = m_a_in * sgn(m_lv[i] - m_lv[i-1]);
        m_bt[i] = (m_lv[i] - m_lv[i-1]) / m_ba[i];
        if(i>1) {
            m_lt[i-1] = m_dt[i-1] - (0.5 * m_bt[i-1]) - (0.5 * m_bt[i]);
        }
    }

    //Linear Segment times at limits
    m_lt[0] = m_dt[0] - m_bt[0] - (0.5 * m_bt[1]);
    m_lt[N-2] = m_dt[N-2] - m_bt[N-1] - (0.5 * m_bt[N-2]);
}

void MotionProfile::GetSegments(std::vector<double> &segment_t, std::vector<double> &segment_q,
                                std::vector<double> &segment_v, std::vector<double> &segment_a)
{
    segment_t.resize(N*2);
    segment_q.resize(N*2);
    segment_v.resize(N*2);
    segment_a.resize(N*2);

    //First listed values are directly the receiced via point information
    segment_t[0] = m_t[0];
    segment_q[0] = m_q[0];
    segment_v[0] = 0.0;
    segment_a[0] = 0.0;

    for(size_t i=0; i<N; ++i) {
        //Blends
        size_t b_idx = i*2+1;
        if(i==0) {
            segment_v[b_idx] = 0;
        }
        else {
            segment_v[b_idx] = m_lv[i-1];
        }
        segment_a[b_idx] = m_ba[i];
        segment_t[b_idx] = segment_t[b_idx-1] + m_bt[i];
        segment_q[b_idx] = segment_q[b_idx-1] + (segment_v[b_idx] * m_bt[i]) + (0.5 * segment_a[b_idx] * pow(m_bt[i],2));
        //Linear Seg
        if(i<N) {
            size_t l_idx = i*2+2;
            segment_v[l_idx] = m_lv[i];
            segment_t[l_idx] = segment_t[l_idx-1] + m_lt[i];
            segment_q[l_idx] = segment_q[l_idx-1] + (segment_v[l_idx] * m_lt[i]);
        }
    }
}
