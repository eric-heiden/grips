#include <ompl/base/Planner.h>
#include <base/PlannerUtils.hpp>
#include <chrono>

#include "AbstractPlanner.hpp"
#include "ThetaStar.h"

#define DEBUG_LISTS 0
#define DEBUG_LIST_LENGTHS_ONLY 0
#define INTERACTIVE_STEP_THROUGH 0
#define STEP_DELAY 0
//#define TESTM 0


ThetaStar::ThetaStar() : AbstractPlanner(), ob::Planner(ss->getSpaceInformation(), "Theta*")
{
    curr_traj = new Trajectory();

    srand((unsigned int) (time(nullptr)));

    /// Euclidean Cost
    COST_SEARCH = true;

    // USE_ASTAR
    USE_ASTAR = false;

    _planningTime = 0;

    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(ss->getSpaceInformation()));
}

ThetaStar::ThetaStar(bool astar, std::string name) : AbstractPlanner(), ob::Planner(ss->getSpaceInformation(), name)
{
    curr_traj = new Trajectory();

    srand((unsigned int) (time(nullptr)));

    /// Euclidean Cost
    COST_SEARCH = true;

    // USE_ASTAR
    USE_ASTAR = astar;

    _planningTime = 0;

    pdef_ = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(ss->getSpaceInformation()));
}

ThetaStar::~ThetaStar()
{
}

/// ==================================================================================
/// initialize()
/// Method to initialize all the ROS related topics
/// ==================================================================================


bool ThetaStar::initialize()
{
//    _publisher.initialize();
//    _subscriber.initialize();
    return true;
}

/// ==================================================================================
/// search(std::vector<std::vector<GNode> >& paths, GNode st)
/// Theta* search
/// ==================================================================================
bool ThetaStar::search(std::vector<std::vector<GNode> > &paths, GNode start, GNode goal)
{
    paths.clear();

    OMPL_INFORM("Start: %d, %d --- Goal: %d, %d ", (int) start.x, (int) start.y, (int) goal.x, (int) goal.y);

    /// STL THETA*
    std::vector<GNode> sol;
    std::vector<std::vector<GNode> > path_sol;

    ThetaStarSearch<GNode> thetastarsearch(COST_SEARCH);

    if (USE_ASTAR)
        thetastarsearch.useAstar();

    if (USE_GRANDPARENT)
        thetastarsearch.use_connectGrandParent();

    unsigned int SearchCount = 0;

    const unsigned int NumSearches = 1;

    GNode *p, *lastOpen = nullptr;
    while (SearchCount < NumSearches)
    {
        // Set Start and goal states
        thetastarsearch.SetStartAndGoalStates(start, goal);

        unsigned int SearchState;
        unsigned int SearchSteps = 1;
        do
        {
            // clear highlighting of last best node
//            _publisher.publishOpenNode(lastOpen);

            SearchState = thetastarsearch.SearchStep();

            SearchSteps++;

            if (SearchState != ThetaStarSearch<GNode>::SEARCH_STATE_SEARCHING)
                break;
#if DEBUG_LISTS
            OMPL_INFORM("Step: %d", (int)SearchSteps);

            int len = 0;

            OMPL_INFORM("Open:");
            p = thetastarsearch.GetOpenListStart();
            lastOpen = p;
            if (p == nullptr)
                OMPL_INFORM("No open nodes");
            else
                _publisher.publishOpenNode(p, true);


            while (p)
            {
                len++;

#if !DEBUG_LIST_LENGTHS_ONLY
                //p->PrintNodeInfo();
                if (len > 1)
                    _publisher.publishOpenNode(p);
#endif

                p = thetastarsearch.GetOpenListNext();
            }
            OMPL_INFORM("Open list has %d",len);

            len = 0;

            OMPL_INFORM("Closed");
            p = thetastarsearch.GetClosedListStart();
            while (p)
            {
                len++;

#if !DEBUG_LIST_LENGTHS_ONLY
                //p->PrintNodeInfo();
                _publisher.publishClosedNode(p);
#endif

                p = thetastarsearch.GetClosedListNext();
            }
            OMPL_INFORM("Closed list has %d nodes",len);
#endif

#if DEBUG_LISTS
            ThetaStarSearch<GNode>::Node *node = thetastarsearch.GetCurrentBestRawNode(); //thetastarsearch.GetSolutionStart();
            if (node == nullptr)
                continue;
            sol.push_back(GNode(node->m_UserState.x, node->m_UserState.y, node->m_UserState.theta, node->m_UserState.steer,
                                node->m_UserState.steer_cost, node->m_UserState.costs, node->m_UserState.orientations));
            bool repeating = false;
//            _publisher.publish(_publisher._createNodeMarker(&(node->m_UserState), .1f, .7f, .1f));
            std::vector<GNode> reached;
            reached.push_back(node->m_UserState);
            while ((node = node->parent) && !repeating)
            {
                OMPL_INFORM("Found parent at %d %d", node->m_UserState.x, node->m_UserState.y);
                for (GNode &r : reached)
                {
                    if (r.x == node->m_UserState.x && r.y == node->m_UserState.y)
                    {
                        repeating = true;
                        break;
                    }
                }
                if (repeating)
                    break;
                sol.push_back(GNode(node->m_UserState.x_r, node->m_UserState.y_r, node->m_UserState.theta,
                                    node->m_UserState.steer, node->m_UserState.steer_cost, node->m_UserState.costs,
                                    node->m_UserState.orientations));
                reached.push_back(node->m_UserState);
//                pub_open_closed_.publish(_createNodeMarker(&(node->m_UserState), .1f, .7f, .1f));
            }

            _publisher.publishGlobalPath(sol, true);

            OMPL_INFORM("Generated partial path of length %d", (int)sol.size());
//            if (sol.size() > 1)
//                _publisher.publishGlobalTraj(sol, false, 1);

            sol.clear();
#endif

#if INTERACTIVE_STEP_THROUGH
            OMPL_INFORM("Press key to proceed");
            std::cin.get();
#endif

#if STEP_DELAY
            usleep(100000); // in microseconds
#endif
        }
        while (SearchState == ThetaStarSearch<GNode>::SEARCH_STATE_SEARCHING);


        if (SearchState == ThetaStarSearch<GNode>::SEARCH_STATE_SUCCEEDED)
        {
            OMPL_INFORM("Search found goal state");

            GNode *node = thetastarsearch.GetSolutionStart();
            int steps = 0;
            int xs, ys;

            xs = node->x;
            ys = node->y;
            sol.push_back(GNode(xs, ys, node->theta, node->steer, node->steer_cost, node->costs, node->orientations));
//            OMPL_INFORM("Nodes: %d,%d,%f,%f", xs, ys, node->theta, node->steer_cost);
            // cout<<" "<<xs<<" "<<ys<<endl;
            while ((node = thetastarsearch.GetSolutionNext()))
            {
                xs = (int) node->x_r;
                ys = (int) node->y_r;
//                OMPL_INFORM("Solution Node %d, (x_r, y_r, theta_r): (%f,%f,%f)", steps, node->x_r, node->y_r, node->theta);
                sol.push_back(GNode(*node));
                // sol.push_back(GNode(xs,ys,node->theta,node->steer,node->steer_cost, node->costs,node->orientations,node->x_r,node->y_r));
                GNode s(xs, ys, node->theta, node->steer, node->steer_cost, node->costs, node->orientations, node->x_r,
                        node->y_r);
//                OMPL_INFORM("Solution Stored Node %d, (x_r, y_r, theta_r): (%f,%f,%f)", steps, s.x_r, s.y_r, s.theta);

                steps++;
            }

            // Once you're done with the solution you can free the nodes up
            thetastarsearch.FreeSolutionNodes();
        }
        else if (SearchState == ThetaStarSearch<GNode>::SEARCH_STATE_FAILED)
        {
            OMPL_ERROR("Search terminated. Did not find goal state");
        }

        // Display the number of loops the search went through
        OMPL_INFORM("SearchSteps: %d ", (int) SearchSteps);

        SearchCount++;

        thetastarsearch.EnsureMemoryFreed();
    }

//    std::reverse(sol.begin(), sol.end());
    paths.push_back(sol);

    unsigned long path_size = paths.size();
    OMPL_INFORM("Found [ %d ] path(s).", (int)path_size);

    return path_size > 0;
}


/// ============================================================================================
/// getNumRand(), return the number of Random Samples to generate into the Neighbor Set of the
/// current vertex
/// ============================================================================================
//int ThetaStar::getNumRand()
//{
//    return NRAND;
//}

/// ============================================================================================
/// getNumRand(), return the number of Random Samples to generate into the Neighbor Set of the
/// current vertex
/// ============================================================================================
//int ThetaStar::getNumBest()
//{
//    return NBEST;
//}

/// ============================================================================================
/// run_global_planner()
/// Main loop to run the global planner
/// ===========================================================================================
ob::PlannerStatus ThetaStar::run()
{
    ob::ScopedState<> start(ss->getStateSpace());
    start[0] = PlannerSettings::environment->start().x;
    start[1] = PlannerSettings::environment->start().y;
    start[2] = 0;
    ob::ScopedState<> goal(ss->getStateSpace());
    goal[0] = PlannerSettings::environment->goal().x;
    goal[1] = PlannerSettings::environment->goal().y;
    goal[2] = 0;

    pdef_->setStartAndGoalStates(start, goal);

    return ob::Planner::solve(PlannerSettings::PlanningTime);
}

og::PathGeometric ThetaStar::geometricPath() const
{
    static constexpr double MIN_INTERPOLATION_DISTANCE = 3;
    og::PathGeometric path(ss->getSpaceInformation());
    auto gnodes = global_paths[0];
    if (gnodes.empty())
    {
        OMPL_ERROR("The computed path contains no GNodes!");
        return path;
    }
    for (auto &node : gnodes)
    {
        auto *state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
        state->setXY(node.x_r, node.y_r);
        state->setYaw(node.theta);
        path.append(state);
    }
    return path;
//    auto trajectory = PlannerUtils::toSteeredTrajectoryPoints(gnodes);
//    auto *state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
//    state->setXY(gnodes[0].x_r, gnodes[0].y_r);
//    state->setYaw(gnodes[0].theta);
//    path.append(state);
//    double arc = 0;
////    QtVisualizer::drawNodes(gnodes, false, Qt::blue);
//    for (std::size_t i = 1; i < trajectory.size()-1; ++i)
//    {
//        arc += std::sqrt(std::pow(trajectory[i].x - trajectory[i - 1].x, 2.)
//                         + std::pow(trajectory[i].x - trajectory[i - 1].x, 2.));
//        if (arc >= MIN_INTERPOLATION_DISTANCE)
//        {
//            arc = 0;
//            state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
//            state->setXY(trajectory[i].x, trajectory[i].y);
//            state->setYaw(PlannerUtils::slope(trajectory[i-1], trajectory[i]));
//            path.append(state);
////            QtVisualizer::drawNode(trajectory[i].x, trajectory[i].y);
//        }
//    }
//    state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
//    state->setXY(gnodes.back().x_r, gnodes.back().y_r);
//    state->setYaw(gnodes.back().theta);
//    path.append(state);
//    return path;
}

std::vector<GNode> ThetaStar::solutionTrajectory() const
{
    return global_paths[0];
}

std::vector<Tpoint> ThetaStar::solutionPath() const
{
    return PlannerUtils::toSteeredTrajectory(global_paths[0]).getPath();
}

bool ThetaStar::hasReachedGoalExactly() const
{
    return true;
}

double ThetaStar::planningTime() const
{
    return _planningTime;
}

void ThetaStar::setProblemDefinition(const ob::ProblemDefinitionPtr &pdef)
{
    pdef_ = pdef;
}

ob::PlannerStatus ThetaStar::solve(const ob::PlannerTerminationCondition &ptc)
{
    pdef_->clearSolutionPaths();
    pdef_->clearSolutionNonExistenceProof();

    auto *goal = dynamic_cast<ob::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return ob::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return ob::PlannerStatus::INVALID_GOAL;
    }

    auto *goalState = ss->getStateSpace()->allocState();
    goal->sampleGoal(goalState);
    GNode goalNode(goalState->as<ob::SE2StateSpace::StateType>()->getX(),
                   goalState->as<ob::SE2StateSpace::StateType>()->getY());

    auto *startState = pdef_->getStartState(0)->as<ob::SE2StateSpace::StateType>();
    GNode startNode(startState->getX(), startState->getY());

    /// ================================
    /// Run
    ///     - read start pose of the robot
    ///     - run the search
    ///     - publish global path
    /// ================================
    curr_traj->reset();
    global_paths.clear();

    PlannerSettings::steering->clearInternalData();

    OMPL_INFORM("Generate a new global path");
    auto begin_time = std::chrono::system_clock::now();
    search(global_paths, startNode, goalNode);
    std::chrono::duration<double> diff = std::chrono::system_clock::now() - begin_time;
    _planningTime = diff.count();

    OMPL_INFORM("Search finished");
    OMPL_INFORM("Global path size : %d", (int) global_paths[0].size());
    if ((int) global_paths[0].size() == 0)
    {
        OMPL_ERROR("No Path found");
        return ob::PlannerStatus::ABORT;
    }

    for (auto &gnodes : global_paths)
    {
        auto path(std::make_shared<og::PathGeometric>(ss->getSpaceInformation()));
        for (auto &node: gnodes)
        {
            auto *state = ss->getStateSpace()->allocState()->as<ob::SE2StateSpace::StateType>();
            state->setXY(node.x_r, node.y_r);
            state->setYaw(node.theta);
            path->append(state);
        }
        pdef_->addSolutionPath(path, false, 0.0, getName());
    }

    return ob::PlannerStatus::EXACT_SOLUTION;
}
