#pragma once

#include <ompl/base/Planner.h>

#include "utils/Stopwatch.hpp"
#include "base/Trajectory.h"


namespace ob = ompl::base;

class TimedResult : public Stopwatch
{
public:
    std::vector<Tpoint> trajectory;
    ob::PlannerStatus status;

    explicit TimedResult(
            std::vector<Tpoint> trajectory = std::vector<Tpoint>(),
            double time = 0)
        : trajectory(std::move(trajectory))
    {
        this->time = time;
    }
};
