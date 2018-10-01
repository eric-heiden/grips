#pragma once

#include "base/gnode.h"
#include "base/Trajectory.h"
#include "base/PlannerUtils.hpp"

#include "steer_functions/steer_base.h"


class Thetastar_planner;

template<class METRIC>
class TMetric
{
public:
    typedef double(*TrajectoryMetric)(const Trajectory *, double);

    static constexpr double ComparisonTolerance = 1e-6;

    static double evaluate(const Trajectory *trajectory, double dt = 0.1)
    {
        return METRIC::evaluateMetric(trajectory, dt);
    }

    static double evaluate(const std::vector<GNode> &path)
    {
        Trajectory trajectory = PlannerUtils::toSteeredTrajectory(path);
        return METRIC::evaluateMetric(&trajectory, .1);
    }

    /**
     * Compares two trajectories by evaluating this metric on them.
     * @param a Path of GNodes.
     * @param b Path of GNodes.
     * @return 1 if a is better than b, 0 if a and b have the same value, -1 if b is better than a
     */
    static int compare(const std::vector<GNode> &a, const std::vector<GNode> &b)
    {
        double va = evaluate(a), vb = evaluate(b);
        if (std::abs(va - vb) < ComparisonTolerance)
            return 0;
        if (METRIC::MoreIsBetter && va > vb)
            return 1;
        return -1;
    }

    /**
     * Compares two trajectories by evaluating this metric on them.
     * @param a Trajectory.
     * @param b Trajectory.
     * @return 1 if a is better than b, 0 if a and b have the same value, -1 if b is better than a
     */
    static int compare(const Trajectory *a, const Trajectory *b, double dt = 0.1)
    {
        double va = evaluate(a, dt), vb = evaluate(b, dt);
        if (std::abs(va - vb) < ComparisonTolerance)
            return 0;
        if (METRIC::MoreIsBetter && va > vb)
            return 1;
        return -1;
    }
};