#pragma once

#include "base/gnode_base.h"
#include "base/PlannerSettings.h"
#include "base/Trajectory.h"
#include "../steer_base.h"

#include "Dubins.h"


class DubinsSteering :  public Steer_base
{
public:
    DubinsSteering(double rho = PlannerSettings::CarTurningRadius);

    /**
     * Steer method to use during the search.
     */
    bool Steer(GNode_base* parent_node, GNode_base *successor) override;

    /**
     * Steer method to use during the visualization of the final path.
     */
    bool Steer(const GNode_base *parent_node, const GNode_base *successor, Trajectory *traj) override;

    /**
     * Loading parameters, setting up internal data structures, etc.
     */
    void initialize() override;


    Steering::SteeringType type() const override
    {
        return Steering::STEER_TYPE_DUBINS;
    }

private:
    double _rho;
};