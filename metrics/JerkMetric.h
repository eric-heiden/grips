#pragma once

#include <vector>
#include <cmath>

#include "TrajectoryMetric.h"

class JerkMetric : public TMetric<JerkMetric>
{
public:
    static double evaluateMetric(const Trajectory *trajectory, double dt)
    {
        /// Change dt according to the one used in the motion planning unit
        double pjerk = 0;
        double jerk_ = 0;
        double Vmax = 100;
        double T = 0;

        double v_x[60000];
        double v_y[60000];

        double acc_x[60000];
        double acc_y[60000];

        double jerk_x[60000];
        double jerk_y[60000];

        int maxV = 0;
        int j = 0;
        unsigned long vsize = trajectory->getV().size();
        double *vel = new double[vsize];
        for (auto vi : trajectory->getV())
        {
            vel[j] = vi;
            T += dt;
            //    cout<<"Vel_i: "<<vi<<endl;
            j++;
        }

        for (int k = 0; k < vsize; k++)
        {
            if (vel[k] > vel[maxV])
                maxV = k;
        }
        Vmax = vel[maxV];

        std::vector<Tpoint> path = trajectory->getPath();

        if (path.size() > 3)
        {
            for (std::size_t i = 0; i < path.size() - 1; i++)
            {
                v_x[i] = (path[i + 1].x - path[i].x);
                v_y[i] = (path[i + 1].y - path[i].y);
            }

            for (std::size_t i = 0; i < path.size() - 2; i++)
            {
                acc_x[i] = (v_x[i + 1] - v_x[i]);
                acc_y[i] = (v_y[i + 1] - v_y[i]);
            }

            /// For the Jerk, it is better to use a Central difference method!!
            /// (ux)_i=(u_{i+1}-u{i-1})/(2*dx) + O(dx^2)
            for (std::size_t i = 1; i < path.size() - 3; i++)
            {
                jerk_x[i] = (acc_x[i + 1] - acc_x[i - 1]) / (2 * dt);
                jerk_y[i] = (acc_y[i + 1] - acc_y[i - 1]) / (2 * dt);
            }

            for (std::size_t i = 1; i < path.size() - 3; i++)
            {
                pjerk += std::fabs(std::sqrt(jerk_x[i] * jerk_x[i] + jerk_y[i] * jerk_y[i]) * dt);
            }

            jerk_ = -pjerk / (T * Vmax);
        }
        else
            jerk_ = 0;

        //    cout<<"DEBUG: jerk "<<jerk_<<endl;

        // delete(vel);
        delete[] vel;
        return jerk_;
    }

    static const bool MoreIsBetter = false;
};