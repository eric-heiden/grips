#pragma once

#include <vector>
#include <cmath>

#include "TrajectoryMetric.h"

class PathLengthMetric : public TMetric<PathLengthMetric>
{
public:
    static double evaluateMetric(const Trajectory *trajectory, double)
    {
        double xold_, yold_, dist_, dx_, dy_, s;
        xold_ = 0;
        yold_ = 0;
        dist_ = 0;

        std::vector <Tpoint> path = trajectory->getPath();
        /// Save the path!!
        for (std::size_t i = 0; i < path.size(); i++)
        {
            if (i == 0)
            {
                xold_ = path[i].x;
                yold_ = path[i].y;
                continue;
            }

            dx_ = (path[i].x - xold_);
            dy_ = (path[i].y - yold_);

            s = std::sqrt(dx_ * dx_ + dy_ * dy_);

            dist_ += s;

            xold_ = path[i].x;
            yold_ = path[i].y;
            //        cout<<"Distance : "<<dist_<<endl;
        }

        return dist_;
    }

    static const bool MoreIsBetter = false;
};