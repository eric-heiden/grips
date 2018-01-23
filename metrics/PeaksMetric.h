#pragma once

#include <vector>
#include <cmath>

#include "TrajectoryMetric.h"

class PeaksMetric : public TMetric<PeaksMetric>
{
public:
    static const bool MoreIsBetter = false;

    static double evaluateMetric(const Trajectory *trajectory, double dt)
    {
        double pmetric = 0;

        double v_x[60000];
        double v_y[60000];

        double acca[60000];

        /// Integration along the found trajectory
#if TESTM > 0
        std::vector<Tpoint> path = traj->getPath();

        v_x[0]=(path[+1].x-path[0].x)/(dt);
        v_y[0]=(path[+1].y-path[0].y)/(dt);
        int acnt=0;
        for (size_t i = 1; i < path.size()-1; i++)
        {
            v_x[i]=(path[i+1].x-path[i-1].x)/(2*dt);
            v_y[i]=(path[i+1].y-path[i-1].y)/(2*dt);
            acca[i-1]=sqrt(v_x[i]*v_x[i]+v_y[i]*v_y[i]);
            acnt++;
        }



        double myeps=0.015;
        for (size_t i = 1; i < acnt-1; i++) {


            if(acca[i]-acca[i-1]>myeps && acca[i]-acca[i+1]>myeps)
                pmetric++;

            if(acca[i]-acca[i-1]<myeps && acca[i]-acca[i+1]<myeps)
                pmetric++;
        }
#else

        std::vector<Tpoint> path = trajectory->getPath();
        std::vector<double> vvec;

        for (auto va : trajectory->getV())
        {
            vvec.push_back(va);
        }

        int acnt = 0;
        double mean_acc = 0;
        for (std::size_t i = 1; i < path.size() - 1; i++)
        {
            v_x[i] = vvec[i] * std::cos(path[i].z);
            v_y[i] = vvec[i] * std::sin(path[i].z);
            acca[i - 1] = std::sqrt(v_x[i] * v_x[i] + v_y[i] * v_y[i]);
            acnt++;
            mean_acc += std::sqrt(v_x[i] * v_x[i] + v_y[i] * v_y[i]);
        }

        mean_acc = mean_acc / acnt;

        double jerk_x[100000];
        double jerk_y[100000];

        double acc_x[100000];
        double acc_y[100000];
        double acc = 0;

        if (path.size() > 3)
        {
            for (std::size_t i = 1; i < path.size() - 2; i++)
            {
                acc_x[i] = (v_x[i + 1] - v_x[i - 1]) / (2 * dt);
                acc_y[i] = (v_y[i + 1] - v_y[i - 1]) / (2 * dt);
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
                acc = std::sqrt((acc_x[i] * acc_x[i] + acc_y[i] * acc_y[i]));
                if (acc < 0.000001 && (jerk_x[i] < 0 || jerk_y[i] < 0))
                    pmetric++;
            }
        }
        else
        {
            pmetric = 0;
        }

#endif

        unsigned long nend = path.size() - 2;
        int peakkk = 0;
        double T, ratio, eps;
        eps = 0.000001;

        for (int i = 1; i < nend - 1; i++)
        {
            T = acca[i] - acca[i - 1];
            ratio = T / mean_acc;
            if (std::fabs(ratio) > 1)
            {
                peakkk++;
            }
        }

        //    cout<<"DEBUG: Number of Peaks:"<<metric<<endl;
        return peakkk;
    }
};