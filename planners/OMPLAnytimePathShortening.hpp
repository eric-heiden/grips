#pragma once

#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/control/planners/rrt/RRT.h>

#include "OMPLPlanner.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;


template<class PLANNER>
class OMPLAnytimePathShortening : public OMPLPlanner<PLANNER>
{
public:
    bool shortcut{true};
    bool hybridize{true};

    virtual ob::PlannerStatus run()// override
    {
        ob::PlannerPtr planner;
        planner = std::make_shared<og::AnytimePathShortening>(this->ss->getSpaceInformation());
        planner->as<og::AnytimePathShortening>()->setHybridize(hybridize);
        planner->as<og::AnytimePathShortening>()->setShortcut(shortcut);
        ob::PlannerPtr optimizingPlanner(new PLANNER(this->ss->getSpaceInformation()));
        planner->as<og::AnytimePathShortening>()->addPlanner(optimizingPlanner);
        this->ss->setPlanner(planner);
        this->ss->setup();
        auto solved = this->ss->solve(PlannerSettings::PlanningTime);
        std::cout << "OMPL anytime path shortening planning status: " << solved.asString() << std::endl;

        if (solved)
        {
//            ss->simplifySolution(); // TODO define time limit?

#ifdef STEER_REEDS_SHEPP
            // Output the length of the path found
            std::cout
                    << planner->getName()
                    << " found a solution of length "
                    << this->ss->getSolutionPath().length()
                    << " with an optimization objective value of "
                    << this->ss->getSolutionPath().cost(this->ss->getOptimizationObjective()) << std::endl;
#endif

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
    };
};

typedef OMPLAnytimePathShortening<og::RRT> RRTAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::RRTstar> RRTstarAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::RRTstar> RRTsharpAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::InformedRRTstar> InformedRRTstarAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::SORRTstar> SORRTstarAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::BITstar> BITstarAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::FMT> FMTAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::BFMT> BFMTAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::PRM> PRMAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::SBL> SBLAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::PRMstar> PRMstarAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::CForest> CForestAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::EST> ESTAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::SST> SSTAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::KPIECE1> KPIECEAnytimePathShortening;
typedef OMPLAnytimePathShortening<og::STRIDE> STRIDEAnytimePathShortening;