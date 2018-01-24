#pragma once
#include <string>

#include "base/gnode_base.h"
#include "base/Trajectory.h"


namespace Steering
{
    enum SteeringType
    {
        STEER_TYPE_REEDS_SHEPP,
        STEER_TYPE_DUBINS,
        STEER_TYPE_POSQ,
        STEER_TYPE_CLOTHOID,
        STEER_TYPE_LINEAR
    };

    inline std::string to_string(Steering::SteeringType t)
    {
        switch (t)
        {
            case STEER_TYPE_REEDS_SHEPP:
                return "Reeds-Shepp";
            case STEER_TYPE_DUBINS:
                return "Dubins";
            case STEER_TYPE_LINEAR:
                return "Linear";
            case STEER_TYPE_POSQ:
                return "POSQ";
            case STEER_TYPE_CLOTHOID:
                return "Clothoid";
        }
    }
}

// Base class for all the steering functions
class Steer_base
{
public:
	/**
     * Steer method to use during the search.
     */
	virtual bool Steer(GNode_base* parent_node, GNode_base *successor) = 0;

	/**
     * Steer method to use during the visualization of the final path.
     */
	virtual bool Steer(const GNode_base *parent_node, const GNode_base *succ, Trajectory *traj) = 0;

	virtual bool clearInternalData()
	{
		return true;
	}

	/**
	 * Loading parameters, setting up internal data structures, etc.
	 */
	virtual void initialize()
	{}

	/**
     * Find the best cost associated to the key.
     */
	virtual double getBestCost(int xp, int yp, int xs, int ys) const
    {
        return 0;
    }

	/**
     * Get the Yaw angle associated to the key.
     */
	virtual double getBestYaw(int xp, int yp, int xs, int ys) const
    {
        return 0;
    }

    virtual Steering::SteeringType type() const = 0;
};