#pragma once

#include <utility>

#include <ompl/geometric/planners/rrt/SORRTstar.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/AnytimePathShortening.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#ifdef G1_AVAILABLE
    #include "steer_functions/G1Clothoid/ClothoidSteering.hpp"
    #include "steer_functions/G1Clothoid/G1ClothoidStateSpace.h"
#endif

#include "steer_functions/POSQ/POSQStateSpace.h"

#include "base/gnode.h"
#include "base/TimedResult.hpp"


namespace ob = ompl::base;
namespace og = ompl::geometric;

class AbstractPlanner
{
public:
    static const unsigned int BSPLINE_MAX_STEPS = 5;
    static constexpr double BSPLINE_EPSILON = 0.005;

    static const unsigned int SHORTCUT_MAX_STEPS = 0;
    static const unsigned int SHORTCUT_MAX_EMPTY_STEPS = 0;
    static constexpr double SHORTCUT_RANGE_RATIO = 0.33;
    static constexpr double SHORTCUT_SNAP_TO_VERTEX = 0.005;

    static const bool ANYTIME_SHORTCUT = true;
    static const bool ANYTIME_HYBRIDIZE = true;

    virtual ob::PlannerStatus run() = 0;

    virtual std::vector<GNode> solutionTrajectory() const = 0;
    virtual std::vector<Tpoint> solutionPath() const = 0;
    virtual og::PathGeometric geometricPath() const = 0;

    virtual bool hasReachedGoalExactly() const = 0;
    virtual double planningTime() const = 0;

    virtual TimedResult shortcutPath() const
    {
        TimedResult r;
        r.status = ss->getLastPlannerStatus();
        og::PathGeometric path = geometricPath();
//        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.2);
        og::PathSimplifier ps(ss->getSpaceInformation(), ss->getGoal());

        r.start();
        ps.shortcutPath(path, SHORTCUT_MAX_STEPS, SHORTCUT_MAX_EMPTY_STEPS,
                        SHORTCUT_RANGE_RATIO, SHORTCUT_SNAP_TO_VERTEX);
        r.stop();

        // have to reduce resolution
        path.interpolate();
//        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
        for (auto *state : path.getStates())
        {
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            double x=s->getX(), y=s->getY();
            r.trajectory.emplace_back(x, y);
        }
        return r;
    }

    virtual TimedResult smoothBSpline() const
    {
        TimedResult r;
        r.status = ss->getLastPlannerStatus();
        og::PathGeometric path = geometricPath();
//        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.2);
        og::PathSimplifier ps(ss->getSpaceInformation(), ss->getGoal());

        r.start();
        ps.smoothBSpline(path, BSPLINE_MAX_STEPS, BSPLINE_EPSILON);
        r.stop();

        // have to reduce resolution
        path.interpolate();
//        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
        for (auto *state : path.getStates())
        {
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            double x=s->getX(), y=s->getY();
            r.trajectory.emplace_back(x, y);
        }
        return r;
    }

    virtual TimedResult simplifyMax() const
    {
        TimedResult r;
        r.status = ss->getLastPlannerStatus();
        og::PathGeometric path = geometricPath();
        og::PathSimplifier ps(ss->getSpaceInformation(), ss->getGoal());

        r.start();
        ps.simplifyMax(path);
        r.stop();

        path.interpolate();
        for (auto *state : path.getStates())
        {
            const auto *s = state->as<ob::SE2StateSpace::StateType>();
            double x=s->getX(), y=s->getY();
            r.trajectory.emplace_back(x, y);
        }
        return r;
    }

    virtual TimedResult anytimePathShortening()
    {
        ob::PlannerPtr planner;
        planner = std::make_shared<og::AnytimePathShortening>(ss->getSpaceInformation());
        planner->as<og::AnytimePathShortening>()->setHybridize(ANYTIME_HYBRIDIZE);
        planner->as<og::AnytimePathShortening>()->setShortcut(ANYTIME_SHORTCUT);
        ob::PlannerPtr optimizingPlanner(omplPlanner());
        planner->as<og::AnytimePathShortening>()->addPlanner(optimizingPlanner);
        this->ss->setPlanner(planner);
        this->ss->setup();
        auto solved = this->ss->solve(PlannerSettings::PlanningTime);
        std::cout << "OMPL anytime path shortening planning status: " << solved.asString().c_str() << std::endl;

        TimedResult r;
        r.status = solved;
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

            auto path = ss->getSolutionPath();
            path.interpolate();
//        ss->getSpaceInformation()->setStateValidityCheckingResolution(0.05);
            for (auto *state : path.getStates())
            {
                const auto *s = state->as<ob::SE2StateSpace::StateType>();
                double x=s->getX(), y=s->getY();
                r.trajectory.emplace_back(x, y);
            }
        }
        else
            std::cout << "No solution found." << std::endl;

        return r;
    }

protected:
    og::SimpleSetup *ss{nullptr};

    AbstractPlanner()
    {
        ob::RealVectorBounds bounds(2);
        bounds.setLow(0, 0);
        bounds.setLow(1, 0);
        bounds.setHigh(0, PlannerSettings::environment->width());
        bounds.setHigh(1, PlannerSettings::environment->height());
        ob::StateSpacePtr space;

        // Construct the robot state space in which we're planning.
        if (PlannerSettings::steeringType == Steering::STEER_TYPE_REEDS_SHEPP)
            space = ob::StateSpacePtr(new ob::ReedsSheppStateSpace(PlannerSettings::CarTurningRadius));
        else if (PlannerSettings::steeringType == Steering::STEER_TYPE_POSQ)
            space = ob::StateSpacePtr(new POSQStateSpace());
        else if (PlannerSettings::steeringType == Steering::STEER_TYPE_DUBINS)
            space = ob::StateSpacePtr(new ob::DubinsStateSpace(PlannerSettings::CarTurningRadius));
        else if (PlannerSettings::steeringType == Steering::STEER_TYPE_LINEAR)
            space = ob::StateSpacePtr(new ob::SE2StateSpace);
#ifdef G1_AVAILABLE
        else if (PlannerSettings::steeringType == Steering::STEER_TYPE_CLOTHOID)
            space = ob::StateSpacePtr(new G1ClothoidStateSpace());
#endif

//        space->setLongestValidSegmentFraction(0.1);
//        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        space->as<ob::SE2StateSpace>()->setBounds(bounds);

        ss = new og::SimpleSetup(space);
        const ob::SpaceInformationPtr si = ss->getSpaceInformation();

        // Construct a space information instance for this state space
//        si->setStateValidityCheckingResolution(0.005);

        // Set the object used to check which states in the space are valid
//        si->setStateValidityChecker(ob::StateValidityCheckerPtr(new ValidityChecker(si)));
//        si->setup();
        ss->setStateValidityChecker([](const ob::State *state)
                                    {
                                        const auto *s = state->as<ob::SE2StateSpace::StateType>();
                                        double x=s->getX(), y=s->getY();
                                        return !PlannerSettings::environment->occupied(x, y);
                                    });

//        si->setStateValidityCheckingResolution(0.005);
        if (PlannerSettings::steeringType == Steering::STEER_TYPE_POSQ)
        {
            ob::MotionValidatorPtr motionValidator(new POSQMotionValidator(si));
            si->setMotionValidator(motionValidator);
            si->setStateValidityCheckingResolution(0.002);
        }
#ifdef G1_AVAILABLE
        else if (PlannerSettings::steeringType == Steering::STEER_TYPE_CLOTHOID)
        {
            ob::MotionValidatorPtr motionValidator(new G1ClothoidStateSpaceValidator(si));
            si->setMotionValidator(motionValidator);
            si->setStateValidityCheckingResolution(0.03);
            // lower granularity necessary to avoid too densely spaced nodes
            // which causes problems in Clothoid steering
        }
#endif

        // Set our robot's starting state
        ob::ScopedState<> start(space);
        start[0] = PlannerSettings::environment->start().x;
        start[1] = PlannerSettings::environment->start().y;
        start[2] = 0;
        // Set our robot's goal state
        ob::ScopedState<> goal(space);
        goal[0] = PlannerSettings::environment->goal().x;
        goal[1] = PlannerSettings::environment->goal().y;
        goal[2] = 0;

        ss->setStartAndGoalStates(start, goal);

        ob::OptimizationObjectivePtr oo(
                new ob::PathLengthOptimizationObjective(si));
        oo->setCostThreshold(ob::Cost(100000.0)); // TODO finish after first solution has been found
        ss->setOptimizationObjective(oo);
        ss->setup();
    }

protected:
    virtual ob::Planner *omplPlanner() = 0;
};
