#pragma once

#include <vector>
#include <cmath>

#include "gui/QtVisualizer.h"
#include "base/PlannerSettings.h"
#include "TrajectoryMetric.h"


class CurvatureMetric : public TMetric<CurvatureMetric>
{
public:
    /**
     * Computes the maximum curvature of the given trajectory.
     * @param trajectory The trajectory to evaluate.
     * @param planner The planner holding the steering function.
     * @return Maximum curvature.
     */
    static double evaluateMetric(const Trajectory *trajectory, double, bool visualize=false)
    {
//        return evaluateMetricOLD(trajectory, 0.1);
        std::vector<Tpoint> path = trajectory->getPath();

        double x1, x2, x3, y1, y2, y3, v1x, v2x, v1y, v2y, v1, v2;
        double infinity = std::numeric_limits<double>::max();
        double maxK = 0;

        size_t traj_size = path.size();

        // Handles the empty input path, setting curvature to infinity
        if (traj_size == 0)
        {
            maxK = infinity;
            return maxK;
        }

        // Handles the input path of length 1 or 2, setting curvature to 0
        if (traj_size < 3)
            return 0;

        // We can compute the curvature in all the points of the path
        // except the first and the last one
        for (int i = 0; i < (traj_size - 2); i++)
        {
            // skip by 2 two steps in both directions
            // to better catch abrupt changes in position
            x1 = path[i].x;
            y1 = path[i].y;

            do
            {
                ++i;
                if (i >= path.size())
                    return maxK;
                x2 = path[i].x;
                y2 = path[i].y;
            }
            while (distance(x1, y1, x2, y2) < 0.3);

            do
            {
                ++i;
                if (i >= path.size())
                    return maxK;
                x3 = path[i].x;
                y3 = path[i].y;
            }
            while (distance(x2, y2, x3, y3) < 0.3);


            // if two points in a row repeat, we skip curvature computation
            if (x1 == x2 && y1 == y2 || x2 == x3 && y2 == y3)
                continue;


            // Infinite curvature in case the path goes a step backwards:
            // p1 - p2 - p1
            if (x1 == x3 && y1 == y3)
            {
                OMPL_WARN("Undefined curvature. Skipping three steps...");
                continue;
            }

            // Compute center of circle that goes through the 3 points
            double cx = (std::pow(x3, 2.) * (-y1 + y2) + std::pow(x2, 2.) * (y1 - y3) -
                         (std::pow(x1, 2.) + (y1 - y2) * (y1 - y3)) * (y2 - y3)) /
                        (2. * (x3 * (-y1 + y2) + x2 * (y1 - y3) + x1 * (-y2 + y3)));
            double cy = (-(std::pow(x2, 2.) * x3) + std::pow(x1, 2.) * (-x2 + x3) +
                         x3 * (std::pow(y1, 2.) - std::pow(y2, 2.)) +
                         x1 * (std::pow(x2, 2.) - std::pow(x3, 2.) + std::pow(y2, 2.) - std::pow(y3, 2.)) +
                         x2 * (std::pow(x3, 2.) - std::pow(y1, 2.) + std::pow(y3, 2.))) /
                        (2. * (x3 * (y1 - y2) + x1 * (y2 - y3) + x2 * (-y1 + y3)));

            // Curvature = 1/Radius
            double radius = std::sqrt(std::pow(x1 - cx, 2.) + std::pow(y1 - cy, 2.));
            double ki = 1. / radius;

#ifdef DEBUG
            if (visualize && ki > 0.5)
            {
                QtVisualizer::drawNode(x1, y1, QColor(255, std::max(0, (int)(255 - ki*10)), 0, 50), .8);
                QtVisualizer::drawNode(x2, y2, QColor(255, std::max(0, (int)(255 - ki*10)), 0, 50), .8);
                QtVisualizer::drawNode(x3, y3, QColor(255, std::max(0, (int)(255 - ki*10)), 0, 50), .8);
            }
#endif

            if (ki > maxK)
                maxK = ki;
        }

        return maxK;
    }

    static double evaluateMetric(std::vector<double> traj_x,
                                 std::vector<double> traj_y)
    {
        double x1, x2, x3, y1, y2, y3, v1x, v2x, v1y, v2y, v1, v2, k_i;
        double infinity = std::numeric_limits<double>::max();
        double maxK = 0;

        size_t traj_size = traj_x.size();

        // Handles the empty input path, setting curvature to infinity
        if (traj_size == 0)
        {
            maxK = infinity;
            return maxK;
        }

        // Handles the input path of length 1 or 2, setting curvature to 0
        if (traj_size < 3)
            return 0;

        // We can compute the curvature in all the points of the path
        // except the first and the last one
        for (int i = 0; i < (traj_size - 2); i++)
        {
            x1 = traj_x[i];
            x2 = traj_x[i + 1];
            x3 = traj_x[i + 2];

            y1 = traj_y[i];
            y2 = traj_y[i + 1];
            y3 = traj_y[i + 2];

            // if two points in a row repeat, we skip curvature computation
            if (x1 == x2 && y1 == y2 || x2 == x3 && y2 == y3)
                continue;


            // Infinite curvature in case the path goes a step backwards:
            // p1 - p2 - p1
            if (x1 == x3 && y1 == y3)
            {
                OMPL_WARN("Undefined curvature. Skipping three steps...");
                continue;
            }

            // Normalization of vectors p2 -> p1 and p2 -> p3 to length 1.
            v1x = x1 - x2;
            v1y = y1 - y2;
            v2x = x3 - x2;
            v2y = y3 - y2;
            v1 = sqrt(v1x * v1x + v1y * v1y);
            v2 = sqrt(v2x * v2x + v2y * v2y);
            v1x = (0.5 * v1x * (v1 + v2)) / v1;
            v1y = (0.5 * v1y * (v1 + v2)) / v1;
            v2x = (0.5 * v2x * (v1 + v2)) / v2;
            v2y = (0.5 * v2y * (v1 + v2)) / v2;

            x1 = x2 + v1x;
            y1 = y2 + v1y;
            x3 = x2 + v2x;
            y3 = y2 + v2y;

            // curvature computation
            k_i = 2 * fabs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) /
                  (sqrt(((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) *
                        ((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1)) *
                        ((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2))));

            if (k_i > maxK)
                maxK = k_i;
        }

        return maxK;
    }

    static const bool MoreIsBetter = false;

private:
    static double distance(double x1, double y1, double x2, double y2)
    {
        return std::sqrt(std::pow(x2-x1, 2.) + std::pow(y2-y1, 2.));
    }
};