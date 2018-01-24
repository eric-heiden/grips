#pragma once

#include "base/PlannerUtils.hpp"
#include "AbstractPlanner.hpp"

/**
 * Derived from AbstractPlanner, this class is intended to simplify
 * existing (hand-crafted) trajectories using OMPL path simplifiers.
 */
class OMPLSimplifier : public AbstractPlanner
{
public:
    explicit OMPLSimplifier(const std::vector<GNode> &path)
            : _path(path), AbstractPlanner()
    {
        _steered = PlannerUtils::toSteeredTrajectory(path).getPath();
        _geometric = new og::PathGeometric(ss->getSpaceInformation());
        for (auto &node: path)
        {
            auto *state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
            state->setXY(node.x_r, node.y_r);
            state->setYaw(node.theta);
            _geometric->append(state);
        }
    }

    ~OMPLSimplifier()
    {
        delete _geometric;
    }

    ob::PlannerStatus run()
    {
        OMPL_ERROR("OMPLSimplifier::run() has not been implemented.");
        return ob::PlannerStatus::CRASH;
    }

    std::vector<GNode> solutionTrajectory() const {return _path;}
    std::vector<Tpoint> solutionPath() const {return _steered;}
    og::PathGeometric geometricPath() const {return *_geometric;}

    bool hasReachedGoalExactly() const {return true;}

    double planningTime() const {return 0;}
    ob::Planner *omplPlanner() {return nullptr;}

private:
    const std::vector<GNode> _path{};
    std::vector<Tpoint> _steered{};
    og::PathGeometric *_geometric{};
};
