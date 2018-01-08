#ifndef TMOTIONPARAMS_H_
#define TMOTIONPARAMS_H_

#include "motionUtils.h"
#include "TMotionPose2D.h"
#include "motionEnums.h"

namespace motion
{
    struct TMotionParams
    {
        TMotionPose2D PT_loc;  /**< Target location. */
        EPTMode PT_mode;  /**< Point tracking mode: LPT(linear) or APT(arc). */
        EPTBumpMode PT_bumpMode;  /**< Bump mode: fragile or strong. */
        double WF_V;  /**< Wall following linear velocity. */
        double WF_dist;  /**< Wall following side distance. */

        TMotionParams(double x = 0.0,
                      double y = 0.0,
                      EPTMode m = LPT,
                      EPTBumpMode bm = FRAGILE,
                      double v = wallFolV,
                      double dist = wallFolSideDist):PT_loc(x, y),
                                                     PT_mode(m),
                                                     PT_bumpMode(bm),
                                                     WF_V(v),
                                                     WF_dist(dist) {}

        inline void reset(void)
        {
            this->PT_loc = TMotionPose2D(0.0, 0.0);
            this->PT_mode = LPT;
            this->PT_bumpMode = FRAGILE;
            this->WF_V = wallFolV;
            this->WF_dist = wallFolSideDist;
        }

        inline const TMotionParams& operator=(const TMotionParams& rhs)
        {
            PT_loc = rhs.PT_loc;
            PT_mode = rhs.PT_mode;
            PT_bumpMode = rhs.PT_bumpMode;
            WF_V = rhs.WF_V;
            WF_dist = rhs.WF_dist;
            return *this;
        }
    };
}
#endif /** TMOTIONPARAMS_H_ */
