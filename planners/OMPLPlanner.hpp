#pragma once

#include <memory>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/cforest/CForest.h>
#include <ompl/geometric/planners/fmt/FMT.h>
#include <ompl/geometric/planners/fmt/BFMT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/est/EST.h>
#include <ompl/geometric/planners/sbl/SBL.h>
#include <ompl/geometric/planners/sst/SST.h>
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/stride/STRIDE.h>
#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/PathSimplifier.h>

#include "base/PlannerSettings.h"
#include "base/PlannerUtils.hpp"
#include "base/gnode.h"
#include "steer_functions/POSQ/POSQStateSpace.h"
#include "planners/AbstractPlanner.hpp"

namespace ob = ompl::base;
namespace og = ompl::geometric;


template<class PLANNER>
class OMPLPlanner : public AbstractPlanner
{
public:
    OMPLPlanner() = default;

    ob::PlannerStatus run() override
    {
        const ob::SpaceInformationPtr si = ss->getSpaceInformation();
        _omplPlanner = ob::PlannerPtr(new PLANNER(si));
        ss->setPlanner(_omplPlanner);
        ss->setup();

        auto solved = ss->solve(PlannerSettings::PlanningTime);
        OMPL_INFORM("OMPL %s planning status: %s",
                    _omplPlanner->getName().c_str(),
                    solved.asString().c_str());

        if (solved)
        {
//            ss->simplifySolution(); // TODO define time limit?
            // Output the length of the path found
            OMPL_INFORM(
                    "%s found a solution of length %f with an optimization objective value of %f",
                    _omplPlanner->getName().c_str(),
                    ss->getSolutionPath().length(),
                    ss->getSolutionPath().cost(ss->getOptimizationObjective())
            );
        }
        else
            OMPL_WARN("No solution found.");
        return solved;
    }

    virtual ~OMPLPlanner()
    {
        delete ss;
    }

    std::vector<GNode> solutionTrajectory() const override
    {
        std::vector<GNode> gnodes;
        og::PathGeometric path = ss->getSolutionPath();
        auto &states = path.getStates();
        for (auto *state : path.getStates())
        {
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            double x=s->getX(), y=s->getY();
            gnodes.emplace_back(GNode(x, y, s->getYaw()));
        }
        PlannerUtils::updateAngles(gnodes, true);
        return gnodes;
    }

    std::vector<Tpoint> solutionPath() const override
    {
        og::PathGeometric path = ss->getSolutionPath();
        path.interpolate();
        std::vector<Tpoint> points;
        for (auto *state : path.getStates())
        {
            // Extract the robot's (x,y) position from its state
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            double x = s->getX(), y = s->getY();
            points.emplace_back(x, y);
        }
        return points;
    }

    bool hasReachedGoalExactly() const override
    {
        return ss->haveExactSolutionPath();
    }

    inline og::PathGeometric geometricPath() const override
    {
        return ss->getSolutionPath();
    }

    double planningTime() const override
    {
        return ss->getLastPlanComputationTime();
    }

protected:
    ob::Planner *omplPlanner() override
    {
        return _omplPlanner.get();
    }

private:
    ob::PlannerPtr _omplPlanner;
};

typedef OMPLPlanner<og::RRT> RRTPlanner;
typedef OMPLPlanner<og::RRTstar> RRTstarPlanner;
typedef OMPLPlanner<og::RRTstar> RRTsharpPlanner;
typedef OMPLPlanner<og::InformedRRTstar> InformedRRTstarPlanner;
typedef OMPLPlanner<og::SORRTstar> SORRTstarPlanner;
typedef OMPLPlanner<og::BITstar> BITstarPlanner;
typedef OMPLPlanner<og::FMT> FMTPlanner;
typedef OMPLPlanner<og::BFMT> BFMTPlanner;
typedef OMPLPlanner<og::PRM> PRMPlanner;
typedef OMPLPlanner<og::SBL> SBLPlanner;
typedef OMPLPlanner<og::PRMstar> PRMstarPlanner;
typedef OMPLPlanner<og::CForest> CForestPlanner;
typedef OMPLPlanner<og::EST> ESTPlanner;
typedef OMPLPlanner<og::SST> SSTPlanner;
typedef OMPLPlanner<og::KPIECE1> KPIECEPlanner;
typedef OMPLPlanner<og::STRIDE> STRIDEPlanner;
