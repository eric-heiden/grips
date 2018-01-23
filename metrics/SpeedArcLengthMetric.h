#pragma once

#include <vector>
#include <cmath>

#include "TrajectoryMetric.h"

class SpeedArcLengthMetric : public TMetric<SpeedArcLengthMetric>
{
public:
    static const bool MoreIsBetter = false;

    static double evaluateMetric(const Trajectory *trajectory, double dt)
    {
        double metric = 0;
        double T = 0;

        double pmetric = 0;

        // TODO don't be wasteful...
        double v_x[60000];
        double v_y[60000];

        double acc_x[60000];
        double acc_y[60000];

        /// Variables for the integration

        /// Found the max velocity applied
        double Vmax = 100;
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

        //    cout<<"Debug VMAX: "<<Vmax<<endl;
        //    cout<<"Debug T: "<<T<<endl;

        std::vector<Tpoint> path = trajectory->getPath();

        for (std::size_t i = 0; i < path.size() - 1; i++)
        {
            v_x[i] = (path[i + 1].x - path[i].x) / dt;
            v_y[i] = (path[i + 1].y - path[i].y) / dt;
            v_x[i] = v_x[i] / Vmax;
            v_y[i] = v_y[i] / Vmax;
        }


        for (std::size_t i = 1; i < path.size() - 2; i++)
        {
            acc_x[i] = (v_x[i + 1] - v_x[i - 1]) / (2 * dt);
            acc_y[i] = (v_y[i + 1] - v_y[i - 1]) / (2 * dt);

            pmetric += (std::sqrt((1 / T) * (1 / T) + (acc_x[i] * acc_x[i] + acc_y[i] * acc_y[i])) * dt);
        }

        /// Normalize the metric
        metric = -std::log(pmetric);
        delete[] vel;
        return metric;
    }
};