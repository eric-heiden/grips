#include "LinearSteering.h"

#include <cmath>

#include "base/PlannerSettings.h"
#include "planners/ThetaStar.h"


bool LinearSteering::Steer(const GNode_base *parent_node, const GNode_base *successor, Trajectory *traj)
{
    traj->reset();
    double dx = (successor->x_r - parent_node->x_r);
    double dy = (successor->y_r - parent_node->y_r);
    double size = std::sqrt(dx*dx + dy*dy);
    const double scale = 0.1;
    dx = dx / size * scale;
    dy = dy / size * scale;

    auto steps = (int)(size / std::sqrt(dx*dx + dy*dy));

    for (int j = 0; j <= steps ; ++j)
    {
        traj->addPointEnd(Tpoint(parent_node->x_r + dx * j, parent_node->y_r + dy * j));
        traj->addVelocities(1, 1);
    }
    traj->addPointEnd(Tpoint(successor->x_r, successor->y_r));
    traj->addVelocities(1, 1);

    // TODO check for collision?
    return true;
}

bool LinearSteering::Steer(GNode_base *parent_node, GNode_base *successor)
{
    double dy = successor->y_r - parent_node->y_r;
    double dx = successor->x_r - parent_node->x_r;
    successor->theta = std::atan2(dy, dx);
    for (int i = 0; i < PlannerSettings::numberEdges; ++i)
        successor->orientations[i] = successor->theta;
    successor->steer_cost = (float) std::sqrt(dy * dy + dx * dx);

    // TODO check for collision?
    return GNode_base::line(parent_node, successor);
}
