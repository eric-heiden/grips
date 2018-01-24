#include <ompl/base/ScopedState.h>
#include "POSQSteering.h"


bool POSQSteering::Steer(GNode_base *parent_node, GNode_base *successor)
{
    // TODO check for collision?
    return true;
}

bool POSQSteering::Steer(const GNode_base *parent_node, const GNode_base *succ, Trajectory *traj)
{
    auto *start = _space.allocState()->as<ob::SE2StateSpace::StateType>();
    auto *goal = _space.allocState()->as<ob::SE2StateSpace::StateType>();
    start->setX(parent_node->x_r);
    start->setY(parent_node->y_r);
    start->setYaw(parent_node->theta);
    goal->setX(succ->x_r);
    goal->setY(succ->y_r);
    goal->setYaw(succ->theta);
    double distance;
    std::vector<UnicycleState> states = _posq.steer(start, goal, distance);
    delete start;
    delete goal;

    for (auto &s : states)
    {
        traj->addPointEnd(Tpoint(s.x_, s.y_, s.yaw_));
        traj->addVelocities(1, 1);
    }

    // TODO check for collision?
    return true;
}
