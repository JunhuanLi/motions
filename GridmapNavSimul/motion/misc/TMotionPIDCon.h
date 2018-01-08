#ifndef TMOTIONPIDCON_H_
#define TMOTIONPIDCON_H_

struct TMotionPIDCon
{
    double m_kP;  /**< Proportional constant. */
    double m_kI;  /**< Integral constant. */
    double m_kD;  /**< Derivative constant. */
    double m_il;  /**< Integral limit. */

    double m_output;  /**< Control output. */
    double m_inteErr;  /**< Integral error. */
    double m_deltaErr;  /**< Delta error. */
    double m_preErr;  /**< Previous error used to calculate \sa m_deltaErr. */

    /** Constructor */
    TMotionPIDCon(double kp, double ki, double kd=0):m_kP(kp), m_kI(ki), m_kD(kd) {}

    /** Controller */
    inline double pid(double err)
    {
        m_inteErr += err;
        m_deltaErr = err - m_preErr;

///        if(m_inteErr >= m_il)
///        {
///            m_inteErr = m_il;
///        }

        m_output = m_kP * err + m_kI * m_inteErr + m_kD * m_deltaErr;
        return m_output;
    }
};

#endif /** TMOTIONPIDCON_H_ */
