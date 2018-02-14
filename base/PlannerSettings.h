#pragma once

#include "Environment.h"
#include "steer_functions/steer_base.h"


typedef Steer_base SteerFunction;

//#define DEBUG
#define STATS

struct PlannerSettings
{
public:
    static Environment *environment;

    static int numberEdges;

    // steering function settings
    static Steering::SteeringType steeringType;
    static SteerFunction *steering;

    static void initializeSteering();

    static double CarTurningRadius;

    static constexpr double PlanningTime{30.0};

    static bool VisualizeSmoothing1;
    static bool VisualizeSmoothing2;
    static bool VisualizeSmoothing3;
    static bool VisualizeSmoothing4;

    // GRIPS settings

    /*
     * Minimum distance to be maintained between two consecutive nodes.
     */
    static double gripsMinNodeDistance;
    /*
     * Gradient descent rate.
     */
    static double gripsEta;
    /*
     * Discount factor for gradient descent rate.
     */
    static double gripsEtaDiscount;

    /**
     * Number of gradient descent rounds.
     */
    static unsigned int gripsGradientDescentRounds;
    /**
     * Maximum number of pruning rounds after which the algorithm
     * should terminate.
     */
    static unsigned int gripsMaxPruningRounds;
};
