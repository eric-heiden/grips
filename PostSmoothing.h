#pragma once
#include <fstream>
#include <chrono>

#include "base/TimedResult.hpp"
#include "base/gnode.h"
#include "base/PlannerUtils.hpp"
#include "base/PlannerSettings.h"

#include "gui/QtVisualizer.h"

#include "metrics/PathLengthMetric.h"
#include "metrics/CurvatureMetric.h"


class PostSmoothing
{
public:
    static const bool MINIMIZE_PATHLENGTH = true; // otherwise minimize curvature

    static const bool FIX_COLLISIONS = false;

    enum RoundType
    {
        ROUND_GD,
        ROUND_PRUNING,
        ROUND_ORIGINAL,
        ROUND_UNKOWN
    };

    struct RoundStats
    {
        double pathLength = -1;
        double maxCurvature = -1;
        double time = -1;
        int nodes = -1;
        double medianNodeObstacleDistance = -1;
        double meanNodeObstacleDistance = -1;
        double minNodeObstacleDistance = -1;
        double maxNodeObstacleDistance = -1;
        double stdNodeObstacleDistance = -1;
        double medianTrajObstacleDistance = -1;
        double meanTrajObstacleDistance = -1;
        double minTrajObstacleDistance = -1;
        double maxTrajObstacleDistance = -1;
        double stdTrajObstacleDistance = -1;
        RoundType type = ROUND_UNKOWN;
        TimedResult stopWatch;

        std::string typeName() const {
            switch (type) {
                case ROUND_GD:
                    return "gd";
                case ROUND_PRUNING:
                    return "pruning";
                case ROUND_ORIGINAL:
                    return "original";
                default:
                    return "unknown";
            }
        }
    };

    static int insertedNodes;
    static int pruningRounds;
    static int collisionFixAttempts;
    static int roundsWithCollisionFixAttempts;
    static std::vector<int> nodesPerRound;
    static std::vector<RoundStats> statsPerRound;
    static double smoothingTime;

    static bool smooth(std::vector<GNode> &path, const std::vector<Tpoint> &originalPathIntermediaries);

    static bool smooth(std::vector<GNode> &path)
    {
        auto intermediary = PlannerUtils::toSteeredTrajectoryPoints(path);
        smooth(path, intermediary);
    }

private:
//    static void fixCollision(std::vector<GNode> &path,
//                             const std::vector<Tpoint> &originalPathIntermediaries,
//                             const Tpoint &node,
//                             unsigned int i)
//    {
//        auto closest = PlannerUtils::closestPoint(node, originalPathIntermediaries);
//        GNode repair;
//        if (closest.euclidianDistance(path[i - 1]) < MIN_NODE_DISTANCE)
//        {
//            path[i - 1].x_r = closest.x_r;
//            path[i - 1].y_r = closest.y_r;
////                    path[i-1].theta = closest.theta;
//            repair = path[i - 1];
//        } else if (closest.euclidianDistance(path[i]) < MIN_NODE_DISTANCE)
//        {
//            path[i].x_r = closest.x_r;
//            path[i].y_r = closest.y_r;
////                    path[i].theta = closest.theta;
//            repair = path[i];
//        } else {
//            repair = GNode(closest.x, closest.y, PlannerUtils::slope(path[i - 1], path[i]));
//            path.insert(path.begin() + i, repair);
//        }
//#ifdef DEBUG
//        QtVisualizer::drawNode(repair, Qt::cyan, 0.4);
//#endif
//    }

    static RoundStats roundStats;

    static void beginRound(RoundType type = ROUND_UNKOWN);

    static void endRound(const std::vector<GNode> &path);

    static Stopwatch stopWatch;
};
