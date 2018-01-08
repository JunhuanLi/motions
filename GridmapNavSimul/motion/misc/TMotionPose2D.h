#ifndef TMOTIONPOSE2D_H_
#define TMOTIONPOSE2D_H_

namespace motion
{
    struct TMotionPose2D
    {
        double x;
        double y;
        double phi;

        TMotionPose2D():x(0.0), y(0.0), phi(0.0) {}
        TMotionPose2D(double xx, double yy):x(xx), y(yy) {}
        TMotionPose2D(double xx, double yy, double pphi):x(xx), y(yy), phi(pphi) {}

        inline const TMotionPose2D& operator=(const TMotionPose2D& rhs)
        {
            x = rhs.x;
            y = rhs.y;
            phi = rhs.phi;
            return *this;
        }

        inline TMotionPose2D operator+(const TMotionPose2D& rhs)
        {
            return TMotionPose2D(x + rhs.x, y + rhs.y, phi + rhs.phi);
        }

        inline TMotionPose2D operator-(const TMotionPose2D& rhs)
        {
            return TMotionPose2D(x - rhs.x, y - rhs.y, phi - rhs.phi);
        }

        inline void reset(void) { x = 0.0; y = 0.0; phi = 0.0; }
    };
}

#endif /** TMOTIONPOSE2D_H_ */
