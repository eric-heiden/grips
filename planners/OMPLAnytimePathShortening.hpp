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

    virtual ob::PlannerStatus run()
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
        OMPL_INFORM("OMPL anytime path shortening planning status: %s", solved.asString().c_str());

        if (!solved)
            OMPL_WARN("OMPL Anytime PS found no solution.");
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