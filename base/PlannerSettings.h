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
    static double Kv;
    static double Ktheta;
    static double Krho;
    static double rhoEndcondition;

    static void initializeSteering();

    // grid environment settings
    static double gridWidth;
    static double gridHeight;
    // TODO support following grid settings
    static double minx;
    static double miny;
    static double cellWidth;
    static double cellHeight;

    // collision checker settings
    static double radiusCollision;
    static int displayInfoCollision;

    static double CarTurningRadius;

    static constexpr double PlanningTime{30.0};

    static bool VisualizeSmoothing1;
    static bool VisualizeSmoothing2;
    static bool VisualizeSmoothing3;
    static bool VisualizeSmoothing4;
};
