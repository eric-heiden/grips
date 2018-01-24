#pragma once

#include "steer_functions/steer_base.h"
#include "POSQ.hpp"


class POSQSteering : public Steer_base
{
public:
    POSQSteering() = default;

    /**
     * Steer method to use during the search.
     */
    bool Steer(GNode_base* parent_node, GNode_base *successor) override;

    /**
     * Steer method to use during the visualization of the final path.
     */
    bool Steer(const GNode_base *parent_node, const GNode_base *succ, Trajectory *traj) override;

    Steering::SteeringType type() const override
    {
        return Steering::STEER_TYPE_POSQ;
    }

    POSQ _posq;
private:
    ob::SE2StateSpace _space;
};
