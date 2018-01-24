#pragma once

#include <algorithm>
#include <ompl/geometric/PathGeometric.h>

#include "planners/stl_thetastar.h"
#include "base/Trajectory.h"
#include "base/Environment.h"

#include "AbstractPlanner.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;


class ThetaStar : public AbstractPlanner, public ob::Planner
{
public:
    ThetaStar();
    virtual ~ThetaStar();

    bool initialize();

    void setProblemDefinition(const ob::ProblemDefinitionPtr &pdef) override;
    ob::PlannerStatus solve(const ob::PlannerTerminationCondition &ptc) override;

    ob::PlannerStatus run() override;

    std::vector<GNode> solutionTrajectory() const override;
    std::vector<Tpoint> solutionPath() const override;
    og::PathGeometric geometricPath() const override;

    bool hasReachedGoalExactly() const override;
    double planningTime() const override;

private:
    Trajectory *curr_traj;

    bool COST_SEARCH;
    bool USE_ASTAR;
    bool USE_GRANDPARENT;

    double _planningTime;

    std::vector<std::vector<GNode> > global_paths;

    bool search(std::vector<std::vector<GNode> > &paths, GNode start, GNode goal);

protected:
    explicit ThetaStar(bool astar, std::string name);

    inline ob::Planner *omplPlanner() override
    {
        return this;
    }
};
