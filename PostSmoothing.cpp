#include "PostSmoothing.h"

int PostSmoothing::insertedNodes = 0;
int PostSmoothing::pruningRounds = 0;
int PostSmoothing::collisionFixAttempts = 0;
int PostSmoothing::roundsWithCollisionFixAttempts = 0;
std::vector<int> PostSmoothing::nodesPerRound;
std::vector<PostSmoothing::RoundStats> PostSmoothing::statsPerRound;
PostSmoothing::RoundStats PostSmoothing::roundStats;
double PostSmoothing::smoothingTime = 0;
Stopwatch PostSmoothing::stopWatch;

bool PostSmoothing::smooth(std::vector<GNode> &path, const std::vector<Tpoint> &originalPathIntermediaries)
{
    const bool AverageAngles = true;

    insertedNodes = 0;
    pruningRounds = 0;
    collisionFixAttempts = 0;
    roundsWithCollisionFixAttempts = 0;
    nodesPerRound.clear();
    statsPerRound.clear();

    // register original path statistics
    beginRound(ROUND_ORIGINAL);
    endRound(path);

    smoothingTime = 0;
    stopWatch.start();

    PlannerUtils::updateAngles(path, AverageAngles);

    double dx, dy;
    double eta = PlannerSettings::gripsEta; // gradient descent step size
    for (int round = 0; round < PlannerSettings::gripsGradientDescentRounds; ++round)
    {
        beginRound(ROUND_GD);
        // gradient descent along distance field
        for (int i = 1; i < path.size()-1; ++i)
        {
            // compute gradient
            PlannerSettings::environment->distanceGradient(path[i].x_r, path[i].y_r, dx, dy, 1.);
            double distance = PlannerSettings::environment->bilinearDistance(path[i].x_r, path[i].y_r);
            distance = std::max(.1, distance);
            path[i].x_r -= eta * dx / distance;
            path[i].y_r += eta * dy / distance;
        }
        eta *= PlannerSettings::gripsEtaDiscount; // discount factor

        PlannerUtils::updateAngles(path, AverageAngles);

#ifdef DEBUG
        QtVisualizer::drawTrajectory(path, QColor(150, 150, 150, 200));
        QtVisualizer::drawNodes(path, false, QColor(70, 180, 250, 150), .2);
#endif

        // add/remove nodes if necessary
        auto tpath = PlannerUtils::toSteeredTrajectoryPoints(path[0], path[1]);
        double lastDistance = PlannerSettings::environment->bilinearDistance(tpath[0].x, tpath[0].y);
        double lastDistance2 = PlannerSettings::environment->bilinearDistance(tpath[1].x, tpath[1].y);
        double lastDifference = lastDistance2 - lastDistance;
        std::vector<GNode> npath;
        Tpoint lastNodePosition(tpath[0].x, tpath[0].y);

        for (int i = 0; i < path.size() - 1; ++i)
        {
            auto current = new GNode(path[i]);
            auto next = new GNode(path[i + 1]);

            lastNodePosition = Tpoint(current->x_r, current->y_r);
            Tpoint nextNodePosition = Tpoint(next->x_r, next->y_r);

            npath.push_back(path[i]);

            tpath = PlannerUtils::toSteeredTrajectoryPoints(path[i], path[i+1]);

            for (auto &p : tpath)
            {
                double distance = PlannerSettings::environment->bilinearDistance(p.x, p.y);
                double difference = distance - lastDistance;
                if (lastDifference < 0 && difference > 0
                    && lastNodePosition.distance(p.x, p.y) >= PlannerSettings::gripsMinNodeDistance
                    && nextNodePosition.distance(p.x, p.y) >= PlannerSettings::gripsMinNodeDistance)
                {
                    // local minimum
                    npath.emplace_back(GNode(p.x, p.y));
                    lastNodePosition = Tpoint(p.x, p.y);

                    ++insertedNodes;
#ifdef DEBUG
                    QtVisualizer::drawNode(p.x, p.y, QColor(255, 150, 0, 180), 0.5);
#endif
                }
                lastDifference = difference;
                lastDistance = distance;
            }
        }
        npath.push_back(path[path.size()-1]);
        path = npath;

        PlannerUtils::updateAngles(path, AverageAngles);
        endRound(path);
    }

#ifdef DEBUG
    for (auto &o : originalPathIntermediaries)
        QtVisualizer::drawNode(o, Qt::darkGreen, 0.1);
#endif

    // try to remove nodes
    size_t lastPathLength;
    unsigned int pruningRound = 1;
    int fixes = 0;
    nodesPerRound.push_back((int)(path.size()));
    do
    {
        beginRound(ROUND_PRUNING);
        if (pruningRound >= PlannerSettings::gripsMaxPruningRounds)
        {
            OMPL_ERROR("Giving up pruning after %i rounds. The smoothed trajectory most likely collides.",
                       pruningRound);
            stopWatch.stop();
            smoothingTime = stopWatch.time;
            return false;
        }

        lastPathLength = path.size();
        OMPL_DEBUG("#### PRUNING ROUND %i", pruningRound++);
        ++pruningRounds;

        fixes = 0;

        // determine unremovable nodes
        std::vector<unsigned int> unremovable;

        std::vector<unsigned int> local_unremovable{0};
        for (unsigned int i = 1; i < path.size() - 1; ++i)
        {
            if (PlannerUtils::collides(path[i - 1], path[i + 1]))
            {
                local_unremovable.push_back(i);

#ifdef DEBUG
                OMPL_DEBUG("%i <--> %i WOULD COLLIDE (%.2f %.2f)",
                             i - 1, i + 1, path[i].x_r, path[i].y_r);
#endif
            }
        }
        local_unremovable.push_back((unsigned int) (path.size() - 1));
        unremovable = local_unremovable;

#ifdef DEBUG
        for (auto i : unremovable)
            {
                QtVisualizer::drawNode(path[i].x_r, path[i].y_r, Qt::darkRed, 0.4);
                OMPL_INFORM("UNREMOVABLE %.2f %.2f", path[i].x_r, path[i].y_r);
            }
#endif

        PlannerUtils::updateAngles(path, AverageAngles, true);

#ifdef DEBUG
        for (unsigned int i = 0; i < path.size(); ++i)
        {
            QtVisualizer::drawNode(path[i], QColor(0, 0, 0, 100), 0.3, false);
//            QtVisualizer::drawLabel(std::to_string(i), path[i].x_r + 0.2, path[i].y_r + 0.2);
        }
#endif

        // compute final trajectory
        std::vector<GNode> finalPath;
        for (unsigned int ui = 1; ui < unremovable.size(); ++ui)
        {
            const auto i = unremovable[ui - 1];
            const auto j = unremovable[ui];

            if (finalPath.empty() || path[i] != finalPath.back())
                finalPath.push_back(path[i]);

            if (j - i <= 1)
                continue; // no intermediary nodes

            std::vector<double> distances(j - i + 1, std::numeric_limits<double>::max());
            std::vector<unsigned int> predecessors(j - i + 1);

            for (unsigned int pi = 0; pi < predecessors.size(); ++pi)
                predecessors[pi] = pi == 0 ? 0 : pi-1;

            distances[0] = 0; // source weight is zero

            // run Bellman-Ford to determine best path from source (i) to sink (j)
            for (auto u = i; u <= j - 1; ++u)
            {
                for (auto v = u + 1; v <= j; ++v)
                {
                    if (PlannerUtils::collides(path[u], path[v]))
                        continue; // a break has the same effect for linear steering and would be more efficient

                    double edgeWeight = PathLengthMetric::evaluate(std::vector<GNode>{path[u], path[v]});

#ifdef DEBUG
//                    double dX = path[v].x_r - path[u].x_r;
//                    double dY = path[v].y_r - path[u].y_r;
//                    double rad = std::atan2(dY, dX);
//                    double plusMinus = v % 2 == 0 ? 1 : -1;
//                    double x = (path[u].x_r + path[v].x_r) * 0.5 + plusMinus * std::pow(edgeWeight, .7)*std::cos(M_PI_2 + rad);
//                    double y = (path[u].y_r + path[v].y_r) * 0.5 + plusMinus * std::pow(edgeWeight, .7)*std::sin(M_PI_2 + rad);
//                    QtVisualizer::drawPath(vector<Tpoint>({
//                                                                  Tpoint(path[u].x_r, path[u].y_r),
//                                                                  Tpoint(x, y),
//                                                                  Tpoint(path[v].x_r, path[v].y_r)}), Qt::black);
//                    QtVisualizer::drawLabel(std::to_string(edgeWeight), x-2, y, Qt::black);
#endif
                    if (distances[u - i] + edgeWeight < distances[v - i])
                    {
                        distances[v - i] = distances[u - i] + edgeWeight;
                        predecessors[v - i] = u - i;
                    }
                }
            }

            unsigned int k = j - i;
            auto insertPosition = finalPath.size();
            while (k > 0)
            {
                if (path[k + i] != finalPath.back())
                    finalPath.insert(finalPath.begin() + insertPosition, path[k + i]);
                if (k == predecessors[k])
                {
                    OMPL_ERROR("Failed to prune path due to loop in shortest path.");
                    break;
                }
                k = predecessors[k];
            }
        }
        if (path.back() != finalPath.back())
            finalPath.push_back(path.back());

        path = finalPath;
        nodesPerRound.push_back((int)path.size());
        endRound(path);

        if (lastPathLength != path.size())
            OMPL_DEBUG("Continuing pruning because lastPathLength (%i) != path.size() (%i)",
                     (int)lastPathLength, (int)path.size());
        if (fixes > 0)
            OMPL_DEBUG("Continuing pruning because fixes (%i) > 0", fixes);
    }
    while (lastPathLength != path.size() || fixes > 0);

    stopWatch.stop();
    smoothingTime += stopWatch.time;

#ifdef DEBUG
//    for (auto &n : path)
//            QtVisualizer::drawNode(n, Qt::darkGreen, .1);

//        QtVisualizer::drawTrajectory(path, QColor(200, 0, 180), 3.f);
    QtVisualizer::drawNodes(path, false, Qt::magenta, 0.2f);

    OMPL_INFORM("Path Length after our PS: %f", PathLengthMetric::evaluate(path));
//    OMPL_INFORM("Speed Arc Length: %f", SpeedArcLengthMetric::evaluate(path, this, PlannerSettings::steering));
//    OMPL_INFORM("Peaks: %f", PeaksMetric::evaluate(path, this, PlannerSettings::steering));
#endif
    OMPL_INFORM("Post-smoothing SUCCEEDED after %i pruning rounds.", pruningRound);

    return true;
}

void PostSmoothing::beginRound(PostSmoothing::RoundType type)
{
#ifdef STATS
    roundStats.stopWatch.start();
    roundStats.type = type;
#endif
}

void PostSmoothing::endRound(const std::vector<GNode> &path)
{
#ifdef STATS
    stopWatch.pause();
    static std::vector<double> nodeDistances, trajDistances;
    roundStats.stopWatch.stop();
    roundStats.time = roundStats.stopWatch.time;
    roundStats.pathLength = PathLengthMetric::evaluate(path);
    roundStats.maxCurvature = CurvatureMetric::evaluate(path);
    roundStats.nodes = (int)path.size();
    nodeDistances.clear();
    for (auto &p : path)
        nodeDistances.push_back(PlannerSettings::environment->bilinearDistance(p.x_r, p.y_r));
    roundStats.medianNodeObstacleDistance = stat::median(nodeDistances);
    roundStats.meanNodeObstacleDistance = stat::mean(nodeDistances);
    roundStats.minNodeObstacleDistance = stat::min(nodeDistances);
    roundStats.maxNodeObstacleDistance = stat::max(nodeDistances);
    roundStats.stdNodeObstacleDistance = stat::std(nodeDistances);
    trajDistances.clear();
    for (auto &p : PlannerUtils::toSteeredTrajectoryPoints(path))
        trajDistances.push_back(PlannerSettings::environment->bilinearDistance(p.x, p.y));
    roundStats.medianTrajObstacleDistance = stat::median(trajDistances);
    roundStats.meanTrajObstacleDistance = stat::mean(trajDistances);
    roundStats.minTrajObstacleDistance = stat::min(trajDistances);
    roundStats.maxTrajObstacleDistance = stat::max(trajDistances);
    roundStats.stdTrajObstacleDistance = stat::std(trajDistances);
    statsPerRound.push_back(roundStats);
    stopWatch.resume();
#endif
}
