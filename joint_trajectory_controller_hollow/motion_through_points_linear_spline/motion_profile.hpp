#include <vector>

class MotionProfile {

public:
    /**
     * @brief MotionProfile Generates a motion path through the passed via-points, obeying to time constraints,
     * using linear interpolation between points, and a parabolic blend to avoid second derivative discontinuities.
     * During the motion, the path followed does not reach the via-points with the timing law specified, but
     * it reaches a point as close, as the specified acceleration allows.
     * @param via_points Points through which the controlled variable should pass.
     * @param time_via_points Time to pass through each of the specified points.
     * @param initial_velocity Initial velocity.
     * @param end_velocity End velocity.
     * @param limit_acceleration Acceleration limit at the start (should be higher than inner).
     * @param inner_acceleration Acceleration limit during the motion (does not require a high value).
     */
    MotionProfile(const std::vector<double> &via_points, const std::vector<double> &time_via_points,
                  const double limit_acceleration, const double inner_acceleration);

private:
    // INPUTS
    unsigned int N; //Number of via points
    std::vector<double> m_q; //via points positions of a joint
    std::vector<double> m_t; //time it should pass at a via point
    double m_a_lim; //Acceleration at limits
    double m_a_in;  //Acceleration at inner segments

    // CALCULATED
    std::vector<double> m_dt;   //time between via-points (N-1)
    std::vector<double> m_bt;   //blend times at via-points (N)
    std::vector<double> m_lt;   //linear segment times (N-1)
    std::vector<double> m_ba;   //blend acceleration (N)
    std::vector<double> m_lv;   //linear segment velocity (N-1)

public:
    /**
     * @brief CalculateSegmentTimes Calculates the times associated with each blend and linear segment
     * along the target trajectory and the associated linear segment velocities, as well as the blend accelerations.
     */
    void CalculateSegmentTimes(void);
    /**
     * @brief GetSegments Retrieve a list of significant times and the associated variable position, velocity and accelerations
     * at these times. The listed times are organized as follows:
     * * @param segment_t List of significant times, organized as follows:
     * segment_t = [t(0) tb(0) tl(0) tb(1) tl(1) ... tb(N)]
     * t(0) time at the first via point
     * tb(0) previous times + time of blend at via point 0
     * tl(0) previous times + time of linear segment between points 0 and 1
     * tb(1) previous times + time of blend at via point 1
     * tl(1) previous times + time of linear segment between points 1 and 2
     * ...
     * @param segment_q List of joint positions calculated at each of the significant times.
     * During a blend segment, the variable is calculated as follows:
     * segment_q = segment_q(i-1) + lv(idx) * (t - t(i)) + 0.5 * ba(idx) * (t - t(i))²
     * During a linear segment, the variable is calculated as follows:
     * segment_q(i) = segment_q(i-1) + lv(idx) * (t - t(i))
     * lv is the linear velocity of the current linear segment index (idx)
     * ba is the blend acceleration of the current blend segment index (idx)
     * @param segment_v first derivative of segment_q
     * @param segment_a second derivative of segment_q
     */
    void GetSegments(std::vector<double> &segment_t, std::vector<double> &segment_q,
                     std::vector<double> &segment_v, std::vector<double> &segment_a);
};
