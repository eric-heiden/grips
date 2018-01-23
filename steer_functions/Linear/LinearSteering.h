#pragma once

#include "steer_functions/steer_base.h"

class LinearSteering : public Steer_base
{
    /**
     * Steer method to use during the search.
     */
    bool Steer(GNode_base* parent_node, GNode_base *successor) override;

    /**
     * Steer method to use during the visualization of the final path.
     */
    bool Steer(const GNode_base *parent_node, const GNode_base *successor, Trajectory *traj) override;

    Steering::SteeringType type() const override
    {
        return Steering::STEER_TYPE_LINEAR;
    }
};