#include "PostSmoothing.h"


int PostSmoothing::insertedNodes = 0;
int PostSmoothing::pruningRounds = 0;
int PostSmoothing::collisionFixAttempts = 0;
int PostSmoothing::roundsWithCollisionFixAttempts = 0;
std::vector<int> PostSmoothing::nodesPerRound;
std::vector<PostSmoothing::RoundStats> PostSmoothing::statsPerRound;
PostSmoothing::RoundStats PostSmoothing::roundStats;
double PostSmoothing::smoothingTime = 0;
std::chrono::time_point<std::chrono::system_clock> PostSmoothing::currentTime;

bool PostSmoothing::
smooth(std::vector<GNode> &path, const std::vector<Tpoint> &originalPathIntermediaries)
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

//    QtVisualizer::drawTrajectory(path, QColor(150, 150, 150, 150));
//    QtVisualizer::drawNodes(path, true, QColor(150, 150, 250, 150));

    smoothingTime = 0;
    currentTime = std::chrono::system_clock::now();

    PlannerUtils::updateAngles(path, AverageAngles);

    double dx, dy;
    double eta = ETA; // gradient descent step size
    for (int round = 0; round < GRADIENT_DESCENT_ROUNDS; ++round)
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
        eta *= ETA_DISCOUNT; // discount factor

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
//                OMPL_INFORM("Difference: %.6f    \t%.2f, %.2f", difference, p.x, p.y);
                if (lastDifference < 0 && difference > 0)
                {
//                    OMPL_INFORM("LOCAL MINIMUM");
                }
                if (lastDifference < 0 && difference > 0
                    && lastNodePosition.distance(p.x, p.y) >= MIN_NODE_DISTANCE
                    && nextNodePosition.distance(p.x, p.y) >= MIN_NODE_DISTANCE)
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

        // merge nodes that are too close together
//        for (int i = 0; i < path.size() - 1; ++i)
//        {
//            auto current = Tpoint(path[i].x, path[i].y);
//            auto next = Tpoint(path[i+1].x, path[i+1].y);
//            if (current.distance(next) <= .5 * MIN_NODE_DISTANCE)
//            {
//                path.erase(path.begin() + i, path.begin() + i + 1);
//                i = 0;
//            }
//        }

        PlannerUtils::updateAngles(path, AverageAngles);
        endRound(path);
    }

    // avoid any collisions until now

//        for (auto &o : originalPathIntermediaries)
//            QtVisualizer::drawNode(o, Qt::darkGreen, 0.1);


    // try to remove nodes
    size_t lastPathLength;
    unsigned int pruningRound = 1;
    int fixes = 0;
    nodesPerRound.push_back((int)(path.size()));
    do
    {
        beginRound(ROUND_PRUNING);
        if (pruningRound >= MAX_PRUNING_ROUNDS)
        {
            OMPL_ERROR("Giving up pruning after %i rounds. The smoothed trajectory most likely collides.",
                       pruningRound);
            std::chrono::duration<double> diff = std::chrono::system_clock::now() - currentTime;
            smoothingTime += diff.count();
            return false;
        }

        lastPathLength = path.size();
        OMPL_INFORM("#### PRUNING ROUND %i", pruningRound++);
        ++pruningRounds;

        fixes = 0;
        if (FIX_COLLISIONS)
        {
            OMPL_INFORM("Attempting to fix collisions...");
            for (unsigned int i = 1; i < path.size(); ++i)
            {
                std::vector<Tpoint> collisions;
                if (PlannerUtils::collides(path[i - 1], path[i], collisions))
                {
#ifdef DEBUG
                    for (auto &point : collisions)
                            QtVisualizer::drawNode(point);
#endif
//                if (collisions.front().distance(collisions.back()) >= MIN_NODE_DISTANCE)
//                {
//                    // come from left and right to repair collisions
//                    fixCollision(path, originalPathIntermediaries, collisions.front(), i);
//                    fixCollision(path, originalPathIntermediaries, collisions.back(), i);
//                }
//                else
//                {
                    // repair via centroid if left and right too close
                    auto centroid = PlannerUtils::centroid(collisions);
                    fixCollision(path, originalPathIntermediaries, centroid, i);
                    ++collisionFixAttempts;
//                }

                    ++i;
                    ++fixes;
                }
            }

            if (fixes > 0)
            {
                eta = ETA; // step size
                for (int round = 0; round < GRADIENT_DESCENT_ROUNDS; ++round)
                {
                    // gradient descent along distance field
                    for (int i = 1; i < path.size() - 1; ++i)
                    {
                        // compute gradient
                        double dx, dy;
                        PlannerSettings::environment->distanceGradient(path[i].x_r, path[i].y_r, dx, dy, 1.);
                        double distance = PlannerSettings::environment->bilinearDistance(path[i].x_r, path[i].y_r);
                        distance = std::max(0.1, distance);
                        path[i].x_r -= eta * dx / distance;
                        path[i].y_r += eta * dy / distance;
                    }
                    eta *= ETA_DISCOUNT; // discount factor
                    PlannerUtils::updateAngles(path, AverageAngles);
                }
                ++roundsWithCollisionFixAttempts;
            }
        }


        // determine unremovable nodes
        std::vector<unsigned int> unremovable;
//        QtVisualizer::drawTrajectory(path);

        std::size_t lastUnremovableSize;
        int unremovableRound = 1;
        lastUnremovableSize = unremovable.size();

//            OMPL_INFORM("Checking unremovables. ROUND %i", unremovableRound++);

        std::vector<unsigned int> local_unremovable{0};
        for (unsigned int i = 1; i < path.size() - 1; ++i)
        {
            if (PlannerUtils::collides(path[i - 1], path[i + 1]))
            {
                local_unremovable.push_back(i);

//            auto tpath = std::vector<GNode>({path[i-1], path[i+1]});
//            for (int j = 0; j < 100; ++j)
//                _publisher.publishGlobalPath(tpath, true, 3);

#ifdef DEBUG
                OMPL_INFORM("%i <--> %i WOULD COLLIDE (%.2f %.2f)",
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

        PlannerUtils::updateAngles(path, AverageAngles, true); // TODO helpful for path length

#ifdef DEBUG
//        for (unsigned int i = 0; i < path.size(); ++i)
//            {
//                QtVisualizer::drawNode(path[i], QColor(0, 0, 0, 100), 0.3, false);
////                QtVisualizer::drawLabel(std::to_string(i), path[i].x_r + 0.2, path[i].y_r + 0.2);
//            }
#endif

        // compute final trajectory
        std::vector<GNode> finalPath;
        for (unsigned int ui = 1; ui < unremovable.size(); ++ui)
        {
            const auto i = unremovable[ui - 1];
            const auto j = unremovable[ui];

            if (finalPath.empty() || path[i] != finalPath.back())
                finalPath.push_back(path[i]);

//            std::cout << "(1) Adding to final path: " << path[i].x_r << " " << path[i].y_r << std::endl;

            if (j - i <= 1)
                continue; // no intermediary nodes

            std::vector<double> distances(j - i + 1, std::numeric_limits<double>::max());
            std::vector<unsigned int> predecessors(j - i + 1);

            for (unsigned int pi = 0; pi < predecessors.size(); ++pi)
                predecessors[pi] = pi == 0 ? 0 : pi-1; // static_cast<unsigned int>(predecessors.size());

            distances[0] = 0; // source weight is zero

            // run Bellman-Ford to determine best path from source (i) to sink (j)
            for (auto u = i; u <= j - 1; ++u)
            {
                for (auto v = u + 1; v <= j; ++v)
                {
                    if (PlannerUtils::collides(path[u], path[v]))
                        continue; // a break has the same effect for linear steering and would be more efficient

                    double edgeWeight;
                    if (MINIMIZE_PATHLENGTH)
                        edgeWeight = PathLengthMetric::evaluate(std::vector<GNode>{path[u], path[v]});
                    else
                        // TODO this is not exactly right, we have to evaluate the full path until v
                        edgeWeight = CurvatureMetric::evaluate(std::vector<GNode>{path[u], path[v]});

#ifdef DEBUG
//                        double dX = path[v].x_r - path[u].x_r;
//                        double dY = path[v].y_r - path[u].y_r;
//                        double rad = std::atan2(dY, dX);
//                        double plusMinus = v % 2 == 0 ? 1 : -1;
//                        double x = (path[u].x_r + path[v].x_r) * 0.5 + plusMinus * std::pow(edgeWeight, .7)*std::cos(M_PI_2 + rad);
//                        double y = (path[u].y_r + path[v].y_r) * 0.5 + plusMinus * std::pow(edgeWeight, .7)*std::sin(M_PI_2 + rad);
//                        QtVisualizer::drawPath(vector<Tpoint>({
//                                                                      Tpoint(path[u].x_r, path[u].y_r),
//                                                                      Tpoint(x, y),
//                                                                      Tpoint(path[v].x_r, path[v].y_r)}), Qt::black);
//                        QtVisualizer::drawLabel(std::to_string(edgeWeight), x-2, y, Qt::black);
#endif
                    if (MINIMIZE_PATHLENGTH)
                    {
                        if (distances[u - i] + edgeWeight < distances[v - i])
                        {
                            distances[v - i] = distances[u - i] + edgeWeight;
                            predecessors[v - i] = u - i;
                        }
                    }
                    else
                    {
                        // TODO this is not exactly right, we have to evaluate the full path until v
                        if (std::max(distances[u - i], edgeWeight) < distances[v - i])
                        {
                            distances[v - i] = std::max(distances[u - i], edgeWeight);
                            predecessors[v - i] = u - i;
                        }
                    }
                }
            }

            unsigned int k = j - i;
            auto insertPosition = finalPath.size();
            while (k > 0)
            {
                if (path[k + i] != finalPath.back())
                    finalPath.insert(finalPath.begin() + insertPosition, path[k + i]);
//                std::cout << "(2) Adding to final path: " << path[k+i].x_r << " " << path[k+i].y_r << std::endl;
//                    OMPL_INFORM("Predecessor[%d] = %d", k + i, predecessors[k] + i);
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

        // visualize end result

        path = finalPath;
        nodesPerRound.push_back((int)path.size());
        endRound(path);
//            break; // TODO remove
        if (lastPathLength != path.size())
            OMPL_INFORM("Continuing pruning because lastPathLength (%i) != path.size() (%i)",
                     (int)lastPathLength, (int)path.size());
        if (fixes > 0)
            OMPL_INFORM("Continuing pruning because fixes (%i) > 0", fixes);
    }
    while (lastPathLength != path.size() || fixes > 0); // TODO make sure this is not an infinite loop

    std::chrono::duration<double> diff = std::chrono::system_clock::now() - currentTime;
    smoothingTime += diff.count();

#ifdef DEBUG
//    for (auto &n : path)
//            QtVisualizer::drawNode(n, Qt::darkGreen, .1);

//        QtVisualizer::drawTrajectory(path, QColor(200, 0, 180), 3.f);
        QtVisualizer::drawNodes(path, false, Qt::magenta, 0.2f);

        OMPL_INFORM("Path Length: %f", PathLengthMetric::evaluate(path));
//    OMPL_INFORM("Speed Arc Length: %f", SpeedArcLengthMetric::evaluate(path, this, PlannerSettings::steering));
//    OMPL_INFORM("Peaks: %f", PeaksMetric::evaluate(path, this, PlannerSettings::steering));
#endif
    OMPL_INFORM("SUCCEEDED after %i pruning rounds.", pruningRound);
    return true;
}

void PostSmoothing::beginRound(PostSmoothing::RoundType type)
{
#ifdef STATS
    roundStats.start = std::chrono::system_clock::now();
    roundStats.type = type;
#endif
}

void PostSmoothing::endRound(const std::vector<GNode> &path)
{
#ifdef STATS
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - currentTime;
    smoothingTime += diff.count();
    static std::vector<double> nodeDistances, trajDistances;
    diff = std::chrono::system_clock::now() - roundStats.start;
    roundStats.time = diff.count();
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
    currentTime = std::chrono::system_clock::now();
#endif
}
