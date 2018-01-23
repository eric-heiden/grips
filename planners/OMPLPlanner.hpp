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

    virtual ob::PlannerStatus run()
    {
        const ob::SpaceInformationPtr si = ss->getSpaceInformation();
        // Construct our optimizing planner using the RRTstar algorithm.
        _omplPlanner = ob::PlannerPtr(new PLANNER(si));
        ss->setPlanner(_omplPlanner);
        ss->setup();

//        // Set the problem instance for our planner to solve
//        optimizingPlanner->setProblemDefinition(pdef);
//        optimizingPlanner->setup();
//        // attempt to solve the planning problem within one second of
//        // planning time
//        auto solved = optimizingPlanner->solve(1.0);

//        auto solved = ss->solve(ob::plannerAlwaysTerminatingCondition());
        auto solved = ss->solve(PlannerSettings::PlanningTime);
        std::cout << "OMPL planning status: " << solved.asString() << std::endl;

        if (solved)
        {
//            ss->simplifySolution(); // TODO define time limit?
            // Output the length of the path found
            std::cout
                    << _omplPlanner->getName()
                    << " found a solution of length "
                    << ss->getSolutionPath().length()
                    << " with an optimization objective value of "
                    << ss->getSolutionPath().cost(ss->getOptimizationObjective()) << std::endl;
//            auto path = boost::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath());
//
//            std::vector<GNode> gnodes;
//            for (auto *state : path->getStates())
//            {
//                const auto * state2D = state->as<ob::SE2StateSpace::StateType>();
//                // Extract the robot's (x,y) position from its state
//                double x = state2D->values[0];
//                double y = state2D->values[1];
//                gnodes.push_back(GNode(x, y));
//            }
//            QtVisualizer::drawPath(gnodes, Qt::blue, 3.f);
        }
        else
            std::cout << "No solution found." << std::endl;
        return solved;
    }

    virtual ~OMPLPlanner()
    {
        delete ss;
    }

    virtual std::vector<GNode> solutionTrajectory() const
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

    virtual std::vector<Tpoint> solutionPath() const
    {
        og::PathGeometric path = ss->getSolutionPath();
        path.interpolate();
        std::vector<Tpoint> points;
        for (auto *state : path.getStates())
        {
//            const auto *state2D = state->as<ob::RealVectorStateSpace::StateType>();
//            // Extract the robot's (x,y) position from its state
//            double x = state2D->values[0];
//            double y = state2D->values[1];
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            double x=s->getX(), y=s->getY();
            points.emplace_back(x, y);
        }
        return points;
//        std::cout << "spath controls: " << ss->getSolutionPath().getControls().size() << std::endl;
//        auto spath = oc::PathControl(ss->getSolutionPath());
//        spath.printAsMatrix(std::cout);
//        std::cout << "spath length: " << spath.length() << std::endl;
//        std::cout << "spath controls: " << spath.getControls().size() << std::endl;
//        og::PathGeometric path = spath.asGeometric();
//        path.interpolate();
//        std::vector<Tpoint> points;
//        for (auto *state : path.getStates())
//        {
////            const auto *state2D = state->as<ob::RealVectorStateSpace::StateType>();
////            // Extract the robot's (x,y) position from its state
////            double x = state2D->values[0];
////            double y = state2D->values[1];
//            const auto *s = state->as<ob::SE2StateSpace::StateType>();
//            double x=s->getX(), y=s->getY();
//            points.emplace_back(x, y);
//        }
//        return points;
    }

    virtual bool hasReachedGoalExactly() const
    {
        return ss->haveExactSolutionPath();
    }

    inline og::PathGeometric geometricPath() const
    {
        return ss->getSolutionPath();
    }

    double planningTime() const
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
