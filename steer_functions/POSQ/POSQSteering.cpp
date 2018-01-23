#include <ompl/base/ScopedState.h>
#include "POSQSteering.h"



//#include "../../metrics/CurvatureMetric.h"
//
//using namespace std;
//
//double POSQSteering::rho_end_condition = 0.05;
//
//int POSQSteering::TYPE;
//int POSQSteering::COST_TYPE;
//double POSQSteering::minYaw;
//double POSQSteering::maxYaw;
//
//POSQSteering::POSQSteering(int *prbdef, const Environment &environment, double kd, double ktheta, double minY, double maxY,
//           int nEdges, double rho, int type, int cost_type)
//        : Ktheta_(ktheta), nEdges_(nEdges), environment_(environment)
//{
//    Kd_ = kd;
//    rho_end_condition = rho;
//    TYPE = type;
//    COST_TYPE = cost_type;
//    minYaw = minY;
//    maxYaw = maxY;
//
//    problemDefinition_ = new int[5];
//    problemDefinition_[0] = prbdef[0];
//    problemDefinition_[1] = prbdef[1];
//    problemDefinition_[2] = prbdef[2];
//    problemDefinition_[3] = prbdef[3];
//    problemDefinition_[4] = prbdef[4];
//
//    srand((unsigned int) time(nullptr));
//
//    traj_x_.clear();
//    traj_y_.clear();
//    traj_th_.clear();
//
//    ROS_INFO("POSQSteering steer function initialized");
//    ROS_INFO("problemDefinition_    %d, %d, %d, %d, %d", problemDefinition_[0], problemDefinition_[1],
//             problemDefinition_[2], problemDefinition_[3], problemDefinition_[4]);
//}
//
//bool POSQSteering::Steer(GNode_base *parent_node, GNode_base *successor)
//{
//    switch (TYPE)
//    {
//        case 0:
//        default:
//            return SteerRandomAngles(parent_node, successor);
//        case 1:
//            return SteerGridAngles(parent_node, successor);
//        case 2:
//            return SteerAnnealing(parent_node, successor);
//        case 3:
//            return SteerStraight(parent_node, successor);
//        case 4:
//            return SteerGridAnglesUSC(parent_node, successor);
//        case 5:
//        case 6:
//        case 7:
//            return SteeringRewire(parent_node, successor);
//        case 9:
//            return SteerRandomFeasible(parent_node, successor);
//        case 10:
//            return SteerDiscreteOptimal(parent_node, successor);
//    }
//}
//
//bool POSQSteering::SteerRandomFeasible(GNode_base *parent_node, GNode_base *successor)
//{
//    double lower_bound = minYaw;
//    double higher_bound = maxYaw;
//
//    double x0, y0, th0, y1, x1;
//    double cost_steer = 0;
//
//    double current_yaw = 0;
//    int n_colls = 0;
//
//    cost_steer = COLLISION_COST;
//
//    int nEdg = getNumEdges();
//
//    /// Connecting to the parent
//    x0 = parent_node->x;
//    y0 = parent_node->y;
//    th0 = parent_node->theta;
//
//    x1 = successor->x;
//    y1 = successor->y;
//
//    for (int i = 0; i < nEdg; ++i)
//    {
//        current_yaw = (lower_bound + (rand()) / ((RAND_MAX / (higher_bound - lower_bound))));
//
//        successor->orientations[i] = current_yaw;
//
//        cost_steer = steerIntegrate(x0, y0, th0, x1, y1, current_yaw);
//
//        if (cost_steer < COLLISION_COST)
//        {
//            break;
//        }
//        else
//        {
//            n_colls++;
//            continue;
//        }
//    }
//
//    if (n_colls < nEdg)
//    {
//        successor->theta = current_yaw;
//        successor->steer_cost = (float) cost_steer;
//        saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, current_yaw);
//        saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, cost_steer);
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}
//
//bool POSQSteering::SteerDiscreteOptimal(GNode_base *parent_node, GNode_base *successor)
//{
//    double lower_bound = minYaw;
//    double higher_bound = maxYaw;
//
//    double x0, y0, th0, y1, x1, th1;
//    double dx, dy;
//    double cost_steer = 0;
//
//    double slice_pi = M_PI / 16;
//
//    int nEdg = (int) (fabs(higher_bound - lower_bound) / slice_pi);
//
//    double current_yaw = 0;
//    double min_cost = COLLISION_COST;
//    int n_colls = 0;
//
//    /// Connecting to the parent
//    x0 = parent_node->x;
//    y0 = parent_node->y;
//    th0 = parent_node->theta;
//
//    x1 = successor->x;
//    y1 = successor->y;
//
//    dy = (y1 - y0);
//    dx = (x1 - x0);
//    th1 = atan2(dy, dx);
//    th1 = normangle(th1, 0);
//
//    double min_theta = th1;
//
//    for (int i = 0; i < nEdg; i++)
//    {
//        successor->orientations[i] = ((double) i) * slice_pi + lower_bound;
//
//        current_yaw = th1 + ((double) i) * slice_pi + lower_bound;
//
//        cost_steer = steerIntegrate(x0, y0, th0, x1, y1, current_yaw);
//
//        if (cost_steer >= COLLISION_COST)
//        {
//            n_colls++;
//            continue;
//        }
//
//        if (cost_steer < min_cost)
//        {
//            min_theta = current_yaw;
//            min_cost = cost_steer;
//        }
//    }
//
//    if (n_colls < nEdg)
//    {
//        successor->theta = min_theta;
//        successor->steer_cost = (float) min_cost;
//        saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, min_theta);
//        saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, min_cost);
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}
//
//bool POSQSteering::SteeringRewire(GNode_base *parent_node, GNode_base *successor)
//{
//    bool res;
//    if (parent_node->parent->x == 0 && parent_node->parent->y == 0)
//    {
//        res = SteerGridAnglesUSC(parent_node, successor);
//        return res;
//    }
//
//    double lower_bound = minYaw;
//    double higher_bound = maxYaw;
//
//    double xp, yp, thp, x0, y0, th0, y1, x1, th1;
//    double dx, dy;
//
//    double cost_steer_gp_to_p = 0;
//    double cost_steer_p_to_curr = 0;
//
//    xp = parent_node->parent->x;
//    yp = parent_node->parent->y;
//    thp = parent_node->parent->theta;
//
//    /// Connecting to the parent
//    x0 = parent_node->x;
//    y0 = parent_node->y;
//
//    dy = (y0 - yp);
//    dx = (x0 - xp);
//
//    th0 = atan2(dy, dx);
//    th0 = normangle(th0, 0);
//
//    x1 = successor->x;
//    y1 = successor->y;
//
//    dy = (y1 - y0);
//    dx = (x1 - x0);
//
//    th1 = atan2(dy, dx);
//    th1 = normangle(th1, 0);
//
//    double angle_rand_one = 0;
//    double angle_rand_two = 0;
//
//    double current_cost = COLLISION_COST;
//    double possible_cost = 0;
//    double min_or[2] = {th0, th1};
//    double min_cost_gp_p = 0;
//    double min_cost_p_curr = 0;
//    int n_coll = 0;
//
//    if (TYPE == 5)
//    {
//        /// Simulated Annealing
//
//        double temp = 1.0;
//        double temp_min = 0.1;
//        double alpha = 0.7;
//        int k = 2;
//        int i;
//        double old_sol[2] = {0, 0};
//        double sol[2] = {th0, th1};
//        double new_sol[2] = {th0, th1};
//
//        double old_c_one = steerIntegrate(xp, yp, thp, x0, y0, sol[0]);
//        double old_c_two = steerIntegrate(x0, y0, sol[0], x1, y1, sol[1]);
//
//        double old_cost[2] = {old_c_one, old_c_two};
//        double new_cost[2] = {0, 0};
//
//        double acc_pr_one = 0;
//        double acc_pr_two = 0;
//
//        while (temp > temp_min)
//        {
//            i = 1;
//
//            while (i < 3)
//            {
//                // new_sol = neighbor(sol)
//                new_sol[0] = sol[0] + temp * (lower_bound + (rand()) / ((RAND_MAX / (higher_bound - lower_bound))));
//                new_sol[1] = sol[1] + temp * (lower_bound + (rand()) / ((RAND_MAX / (higher_bound - lower_bound))));
//
//                //new_cost = cost(new_sol)
//                new_cost[0] = steerIntegrate(xp, yp, thp, x0, y0, new_sol[0]);
//                new_cost[1] = steerIntegrate(x0, y0, new_sol[0], x1, y1, new_sol[1]);
//
//                // ap = acceptance_probability(old_cost, new_cost, T)
//                acc_pr_one = acceptance_probability(old_cost[0], new_cost[0], temp);
//                acc_pr_two = acceptance_probability(old_cost[1], new_cost[1], temp);
//
//                if (acc_pr_one > ((double) rand() / (RAND_MAX)) && acc_pr_two > ((double) rand() / (RAND_MAX)))
//                {
//                    old_sol[0] = sol[0];
//                    old_sol[1] = sol[1];
//
//                    sol[0] = new_sol[0];
//                    sol[1] = new_sol[1];
//
//                    old_cost[0] = new_cost[0];
//                    old_cost[1] = new_cost[1];
//                }
//                i++;
//            }
//            temp = temp * alpha;
//            k++;
//        }
//
//        if (old_cost[0] < COLLISION_COST && old_cost[1] < COLLISION_COST)
//        {
//            th0 = sol[0];
//            th1 = sol[1];
//
//            successor->theta = th1;
//            successor->steer_cost = (float) min_cost_p_curr;
//            saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, successor->theta);
//            saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, successor->steer_cost);
//
//            parent_node->theta = th0;
//            parent_node->steer_cost = (float) min_cost_gp_p;
//            saveBestYaw(parent_node->parent->x, parent_node->parent->y, parent_node->x, parent_node->y,
//                        parent_node->theta);
//            saveBestCost(parent_node->parent->x, parent_node->parent->y, parent_node->x, parent_node->y,
//                         parent_node->steer_cost);
//
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//
//    if (TYPE == 6)
//    {
//        /// Random approach
//        for (int i = 0; i < nEdges_; i++)
//        {
//            angle_rand_one = th0 + (lower_bound + (rand()) / ((RAND_MAX / (higher_bound - lower_bound))));
//            angle_rand_two = th1 + (lower_bound + (rand()) / ((RAND_MAX / (higher_bound - lower_bound))));
//
//            cost_steer_gp_to_p = steerIntegrate(xp, yp, thp, x0, y0, angle_rand_one);
//            cost_steer_p_to_curr = steerIntegrate(x0, y0, angle_rand_one, x1, y1, angle_rand_two);
//
//            if (cost_steer_p_to_curr < COLLISION_COST && cost_steer_gp_to_p < COLLISION_COST)
//            {
//                possible_cost = cost_steer_gp_to_p + cost_steer_p_to_curr;
//
//                if (possible_cost < current_cost)
//                {
//                    min_or[0] = angle_rand_one;
//                    min_or[1] = angle_rand_two;
//                    min_cost_gp_p = cost_steer_gp_to_p;
//                    min_cost_p_curr = cost_steer_p_to_curr;
//                    current_cost = possible_cost;
//                }
//            }
//            else
//            {
//                n_coll++;
//            }
//        }
//
//        if (n_coll < nEdges_)
//        {
//            th0 = min_or[0];
//            th1 = min_or[1];
//
//            successor->theta = th1;
//            successor->steer_cost = (float) min_cost_p_curr;
//            saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, successor->theta);
//            saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, successor->steer_cost);
//
//            parent_node->theta = th0;
//            parent_node->steer_cost = (float) min_cost_gp_p;
//            saveBestYaw(parent_node->parent->x, parent_node->parent->y, parent_node->x, parent_node->y,
//                        parent_node->theta);
//            saveBestCost(parent_node->parent->x, parent_node->parent->y, parent_node->x, parent_node->y,
//                         parent_node->steer_cost);
//
//            return true;
//        }
//        else
//        {
//            return false;
//        }
//    }
//
//    if (TYPE == 7)
//    {
//        //TODO implement
//    }
//
//    return false;
//}
//
//bool POSQSteering::SteerGridAnglesUSC(GNode_base *parent_node, GNode_base *successor)
//{
//    double lower_bound = minYaw;
//    double higher_bound = maxYaw;
//
//    double x0, y0, th0, y1, x1, th1;
//    double dx, dy;
//    double cost_steer = 0;
//
//    int nEdg = getNumEdges();
//    double slice_pi = (2 * M_PI / (double) nEdg);
//    for (int i = 0; i < nEdg; i++)
//    {
//        successor->orientations[i] = ((double) i) * slice_pi;
//    }
//    saveOrientations(parent_node->x, parent_node->y, successor->x, successor->y, successor->orientations);
//
//    /// Connecting to the parent
//    x0 = parent_node->x;
//    y0 = parent_node->y;
//    th0 = parent_node->theta;
//
//    x1 = successor->x;
//    y1 = successor->y;
//
//    dy = (y1 - y0);
//    dx = (x1 - x0);
//    th1 = atan2(dy, dx);
//    th1 = normangle(th1, 0);
//
//    /// could be loaded via params..
//    double discretizations[NDISCRETIZATION] = {M_PI / 4, M_PI / 8, M_PI / 16};
//
//    vector<double> checked_angles;
//
//    double tot_disp_ang = higher_bound - lower_bound;
//
//    int nElements;
//
//    int nSkipped = 0;
//
//    double angle_to_check = 0;
//
//    double current_yaw = 0;
//
//    for (int i = 0; i < NDISCRETIZATION; i++)
//    {
//        nElements = (int) floor(tot_disp_ang / discretizations[i]);
//
//        // ROS_INFO("#nElements %d ", nElements);
//
//        for (int j = 0; j < nElements; j++)
//        {
//            // angle_to_check = ((double)j)*discretizations[i];
//
//            if (j == 0)
//            {
//                angle_to_check = ((double) (nElements) / 2) * discretizations[i];
//            }
//            else
//            {
//                if (j % 2 == 0)
//                {
//                    angle_to_check = ((double) j) * discretizations[i];
//                }
//                else
//                {
//                    angle_to_check = ((double) (nElements - j)) * discretizations[i];
//                }
//            }
//
//            if (std::find(checked_angles.begin(), checked_angles.end(), angle_to_check) != checked_angles.end())
//            {
//                nSkipped++;
//                continue;
//            }
//            else
//            {
//                current_yaw = th1 + angle_to_check + lower_bound;
//
//                cost_steer = steerIntegrate(x0, y0, th0, x1, y1, current_yaw);
//
//                checked_angles.push_back(angle_to_check);
//
//                if (cost_steer < COLLISION_COST)
//                    break;
//            }
//        }
//    }
//
//    // ROS_INFO("#Angles skipped %d ", nSkipped);
//
//    if (cost_steer < COLLISION_COST)
//    {
//        successor->theta = current_yaw;
//        successor->steer_cost = (float) cost_steer;
//        saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, current_yaw);
//        saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, cost_steer);
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}
//
//bool POSQSteering::SteerAnnealing(GNode_base *parent_node, GNode_base *successor)
//{
//    double lower_bound = minYaw;
//    double higher_bound = maxYaw;
//
//    double x0, y0, th0, y1, x1, th1;
//
//    x0 = parent_node->x;
//    y0 = parent_node->y;
//    th0 = parent_node->theta;
//
//    x1 = successor->x;
//    y1 = successor->y;
//
//    int nEdg = getNumEdges();
//    double slice_pi = (2 * M_PI / (double) nEdg);
//    /// Grid specified by nEdg
//    for (int i = 0; i < nEdg; i++)
//    {
//        successor->orientations[i] = ((double) i) * slice_pi;
//    }
//
//    // ThetaStar::saveOrientations(parent_node->x,parent_node->y,successor->x,successor->y, successor->orientations);
//    saveOrientations(parent_node->x, parent_node->y, successor->x, successor->y, successor->orientations);
//
//    double dx, dy;
//    dy = (y1 - y0);
//    dx = (x1 - x0);
//    th1 = atan2(dy, dx);
//    th1 = normangle(th1, 0);
//
//    double old_cost = 0;
//    double new_cost = 0;
//    double acc_pr = 0;
//    double sol = th1;
//    double new_sol;
//    old_cost = steerIntegrate(x0, y0, th0, x1, y1, sol);
//
//    double temp = 1.0;
//    double temp_min = 0.1;
//    double alpha = 0.25;
//    int k = 2;
//    int i;
//    while (temp > temp_min)
//    {
//        i = 1;
//        while (i < 3)
//        {
//            // new_sol = neighbor(sol)
//            new_sol = sol + temp * (lower_bound + (rand()) / ((RAND_MAX / (higher_bound - lower_bound))));
//
//            // new_sol = sol + (sol - old_sol)*((rand()) /(RAND_MAX));
//
//
//            // if (i%2==0) {
//            //     new_sol=atan2(dy,dx)+successor->orientations[i];
//            // }
//            // else{
//            //     new_sol=atan2(dy,dx)+successor->orientations[nEdg-i];
//
//            // }
//
//            //new_cost = cost(new_sol)
//            new_cost = steerIntegrate(x0, y0, th0, x1, y1, new_sol);
//            // ap = acceptance_probability(old_cost, new_cost, T)
//            acc_pr = acceptance_probability(old_cost, new_cost, temp);
//
//            if (acc_pr > ((double) rand() / (RAND_MAX)))
//            {
//                sol = new_sol;
//                old_cost = new_cost;
//            }
//            i++;
//        }
//
//        temp = temp * alpha;
//        // temp = temp*(1/log(k));
//        k++;
//    }
//
//    if (old_cost < COLLISION_COST)
//    {
//        // th1 = atan2(dy/dt,dx/dt)+successor->orientations[min_ind];
//        successor->theta = sol;
//        successor->steer_cost = (float) old_cost;
//        saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, sol);
//        saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, old_cost);
//
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}
//
//bool POSQSteering::SteerStraight(GNode_base *parent_node, GNode_base *successor)
//{
//    double x0, y0, th0, y1, x1, th1;
//    double dx, dy;
//    double cost_steer = 0;
//
//    int nEdg = getNumEdges();
//    double slice_pi = (2 * M_PI / (double) nEdg);
//    /// Grid specified by nEdg
//    for (int i = 0; i < nEdg; i++)
//    {
//        successor->orientations[i] = ((double) i) * slice_pi;
//    }
//
//    // ThetaStar::saveOrientations(parent_node->x,parent_node->y,successor->x,successor->y, successor->orientations);
//    saveOrientations(parent_node->x, parent_node->y, successor->x, successor->y, successor->orientations);
//
//    x0 = parent_node->x;
//    y0 = parent_node->y;
//    th0 = parent_node->theta;
//
//    x1 = successor->x;
//    y1 = successor->y;
//
//    dy = (y1 - y0);
//    dx = (x1 - x0);
//    th1 = atan2(dy, dx);
//    th1 = normangle(th1, 0);
//
//    cost_steer = steerIntegrate(x0, y0, th0, x1, y1, th1);
//
//    if (cost_steer < COLLISION_COST)
//    {
//        successor->theta = th1;
//        successor->steer_cost = (float) cost_steer;
//        saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, th1);
//        saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, cost_steer);
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}
//
//bool POSQSteering::SteerGridAngles(GNode_base *parent_node, GNode_base *successor)
//{
//    double Kd = Kd_;
//    double Ktheta = Ktheta_;
//    double lower_bound = minYaw;
//    double higher_bound = maxYaw;
//
//    double x0, y0, th0, y1, x1, th1 = 0;
//
//    int nEdg = getNumEdges();
//    // double slice_pi = (2*M_PI / (double)nEdg);
//
//    double slice_pi = (2 * (higher_bound - lower_bound) / (double) nEdg);
//
//    /// Grid specified by nEdg
//    for (int i = 0; i < nEdg; i++)
//    {
//        successor->orientations[i] = ((double) i) * slice_pi + lower_bound;
//    }
//
//    // ThetaStar::saveOrientations(parent_node->x,parent_node->y,successor->x,successor->y, successor->orientations);
//    saveOrientations(parent_node->x, parent_node->y, successor->x, successor->y, successor->orientations);
//
//    double min_cost = 100;
//
//    int n_coll = 0;
//
//    int collision = 0;
//
//    double dx, dy, dt;
//
//    int success_steer = 0;
//
//    vector<double> steerstep_res;
//
//    // for (int i=0; i<ThetaStar::getNumEdges();i++){
//    for (int i = 0; i < nEdg; i++)
//    {
//        success_steer = 0;
//
//        collision = 0;
//        successor->steer_cost = 0;
//
//        x0 = parent_node->x;
//        y0 = parent_node->y;
//        th0 = parent_node->theta;
//
//        x1 = successor->x;
//        y1 = successor->y;
//
//        dt = 0.1;
//        dy = (y1 - y0);
//        dx = (x1 - x0);
//        int curr_index = 0;
//
//        if (i == 0)
//        {
//            th1 = atan2(dy / dt, dx / dt);
//        }
//        else
//        {
//            if (i % 2 == 0)
//            {
//                th1 = atan2(dy / dt, dx / dt) + successor->orientations[i];
//                curr_index = i;
//            }
//            else
//            {
//                th1 = atan2(dy / dt, dx / dt) + successor->orientations[nEdg - i];
//                curr_index = nEdg - i;
//            }
//        }
//
//        th1 = normangle(th1, 0);
//        // ROS_INFO("Incoming orientations, parent %f successor %f, theta %f ",parent_node->theta,successor->theta,th1);
//        double sl, sr, oldSl, oldSr, t, eot, dSl, dSr, dSm, dSd, vl, vr, enc_l, enc_r;
//
//        enc_l = 0;
//        enc_r = 0;
//        sl = 0;
//        sr = 0;
//        oldSl = 0;
//        oldSr = 0;
//        eot = 0;
//        t = 0;
//
//        double b, dir;
//
//        b = 0.40;
//        dir = 1;
//
//        double x_int, y_int, th_int, curr_x, curr_y, curr_th;
//
//        x_int = x0;
//        y_int = y0;
//        th_int = th0;
//
//        clearTraj();
//
//        while (eot == 0)
//        {
//            // calculate distance for both wheels
//            dSl = sl - oldSl;
//            dSr = sr - oldSr;
//            dSm = (dSl + dSr) / 2;
//            dSd = (dSr - dSl) / b;
//
//            curr_x = x_int + dSm * cos(th_int + dSd / 2);
//            curr_y = y_int + dSm * sin(th_int + dSd / 2);
//            curr_th = normangle(th_int + dSd, -M_PI);
//
//            steerstep_res = steer_step(curr_x, curr_y, curr_th, x1, y1, th1, t, b, (int) dir);
//
//            eot = steerstep_res[4];
//            vl = steerstep_res[0];
//            vr = steerstep_res[1];
//            //Increase the timer
//            t = t + dt;
//
//            // keep track of previous wheel position
//            oldSl = sl;
//            oldSr = sr;
//
//
//            // increase encoder values
//            enc_l = enc_l + dt * vl;
//            enc_r = enc_r + dt * vr;
//
//            sl = enc_l;
//            sr = enc_r;
//
//            if (COST_TYPE == 0)
//            {
//                successor->steer_cost = (float) (successor->steer_cost + Kd * sqrt((curr_x - x_int) * (curr_x - x_int) +
//                                                                                   (curr_y - y_int) *
//                                                                                   (curr_y - y_int)) +
//                                                 Ktheta * fabs(1 - cos(diff_angle_unwrap(curr_th, th_int))));
//            }
//            else if (COST_TYPE == 2)
//            {
//                successor->steer_cost = (float) (successor->steer_cost + sqrt((curr_x - x_int) * (curr_x - x_int) +
//                                                                              (curr_y - y_int) * (curr_y - y_int)));
//            }
//
//            //save the state for the next sample
//            x_int = curr_x;
//            y_int = curr_y;
//            th_int = curr_th;
//
//            traj_x_.push_back(x_int);
//            traj_y_.push_back(y_int);
//            traj_th_.push_back(th_int);
//
//            if (environment_.collides(x_int, y_int))
//            {
//                collision = 1;
//                n_coll++;
//                successor->steer_cost = -1.0f;
//                successor->costs[curr_index] = -1.0;
//                success_steer = 0;
//                break;
//            }
//        }
//
//        if (COST_TYPE == 1 && collision == 0);
//
//        successor->steer_cost = (float) CurvatureMetric::evaluateMetric(traj_x_, traj_y_);
//
//        if (collision == 0 && curr_index == 0)
//        {
//            // ROS_INFO("N collision in check: %d",n_coll);
//            min_cost = successor->steer_cost;
//            successor->costs[0] = successor->steer_cost;
//
//            success_steer = 1;
//        }
//
//
//        if (collision == 0 && curr_index > 0)
//        {
//
//            successor->costs[curr_index] = successor->steer_cost;
//
//            if (successor->steer_cost < min_cost)
//            {
//                min_cost = successor->steer_cost;
//
//                success_steer = 1;
//            }
//
//        }
//
//
//        if (success_steer == 1)
//            break;
//
//    }
//
//
//
//
//    // if(n_coll<ThetaStar::getNumEdges()){
//
//    if (collision == 0)
//    {
//        // th1 = atan2(dy/dt,dx/dt)+successor->orientations[min_ind];
//        successor->theta = th1;
//        successor->steer_cost = (float) min_cost;
//        saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, th1);
//        saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, min_cost);
//
//        return true;
//    }
//    else
//    {
//        return false;
//    }
//}
//
//bool POSQSteering::SteerRandomAngles(GNode_base *parent_node, GNode_base *successor)
//{
//    double Kd = Kd_;
//    double Ktheta = Ktheta_;
//
//    double x0, y0, th0, y1, x1, th1;
//
//    double lower_bound = minYaw;
//    double higher_bound = maxYaw;
//
//    for (int i = 0; i < getNumEdges(); i++)
//    {
//        successor->orientations[i] = lower_bound + (rand()) / ((RAND_MAX / (higher_bound - lower_bound)));
//    }
//
//    saveOrientations(parent_node->x, parent_node->y, successor->x, successor->y, successor->orientations);
//
//    double min_cost = 100;
//    int min_ind = 0;
//    int n_coll = 0;
//    int collision = 0;
//
//    double dx = 0, dy = 0, dt = 0;
//
//    vector<double> steerstep_res;
//
//    for (int i = 0; i < getNumEdges(); i++)
//    {
//        collision = 0;
//        successor->steer_cost = 0;
//
//        x0 = parent_node->x;
//        y0 = parent_node->y;
//        th0 = parent_node->theta;
//
//        x1 = successor->x;
//        y1 = successor->y;
//
//        dt = 0.1;
//        dy = (y1 - y0);
//        dx = (x1 - x0);
//
//        th1 = atan2(dy / dt, dx / dt) + successor->orientations[i];
//
//        // ROS_INFO("Incoming orientations, parent %f successor %f, theta %f ",parent_node->theta,successor->theta,th1);
//        double sl, sr, oldSl, oldSr, t, eot, dSl, dSr, dSm, dSd, vl, vr, enc_l, enc_r;
//
//        enc_l = 0;
//        enc_r = 0;
//        sl = 0;
//        sr = 0;
//        oldSl = 0;
//        oldSr = 0;
//        eot = 0;
//        t = 0;
//
//        double b, dir;
//
//        b = 0.40;
//        dir = 1;
//
//        double x_int, y_int, th_int, curr_x, curr_y, curr_th;
//
//        x_int = x0;
//        y_int = y0;
//        th_int = th0;
//
//        clearTraj();
//
//        while (eot == 0)
//        {
//            // calculate distance for both wheels
//            dSl = sl - oldSl;
//            dSr = sr - oldSr;
//            dSm = (dSl + dSr) / 2;
//            dSd = (dSr - dSl) / b;
//
//            curr_x = x_int + dSm * cos(th_int + dSd / 2);
//            curr_y = y_int + dSm * sin(th_int + dSd / 2);
//            curr_th = normangle(th_int + dSd, -M_PI);
//
//            steerstep_res = steer_step(curr_x, curr_y, curr_th, x1, y1, th1, t, b, (int) dir);
//
//            eot = steerstep_res[4];
//            vl = steerstep_res[0];
//            vr = steerstep_res[1];
//
//            //Increase the timer
//            t = t + dt;
//
//            // keep track of previous wheel position
//            oldSl = sl;
//            oldSr = sr;
//
//            // increase encoder values
//            enc_l = enc_l + dt * vl;
//            enc_r = enc_r + dt * vr;
//
//            sl = enc_l;
//            sr = enc_r;
//
//            if (COST_TYPE == 0)
//            {
//                successor->steer_cost = (float) (successor->steer_cost + Kd * sqrt((curr_x - x_int) * (curr_x - x_int) +
//                                                                                   (curr_y - y_int) *
//                                                                                   (curr_y - y_int)) +
//                                                 Ktheta * fabs(1 - cos(diff_angle_unwrap(curr_th, th_int))));
//            }
//            else if (COST_TYPE == 2)
//            {
//                successor->steer_cost = (float) (successor->steer_cost + sqrt((curr_x - x_int) * (curr_x - x_int) +
//                                                                              (curr_y - y_int) * (curr_y - y_int)));
//            }
//
//            //save the state for the next sample
//            x_int = curr_x;
//            y_int = curr_y;
//            th_int = curr_th;
//
//            traj_x_.push_back(x_int);
//            traj_y_.push_back(y_int);
//            traj_th_.push_back(th_int);
//
//            if (environment_.collides(x_int, y_int))
//            {
//                collision = 1;
//                n_coll++;
//                successor->steer_cost = -1.0f;
//                successor->costs[i] = -1.0;
//                break;
//            }
//        }
//
//
//        if (COST_TYPE == 1 && collision == 0);
//
//        successor->steer_cost = (float) CurvatureMetric::evaluateMetric(traj_x_, traj_y_);
//
//        if (collision == 0 && i == 0)
//        {
//            // ROS_INFO("N collision in check: %d",n_coll);
//            min_cost = successor->steer_cost;
//            min_ind = i;
//            successor->costs[i] = successor->steer_cost;
//        }
//
//
//        if (collision == 0 && i > 0)
//        {
//            successor->costs[i] = successor->steer_cost;
//
//            if (successor->steer_cost < min_cost)
//            {
//                min_cost = successor->steer_cost;
//                min_ind = i;
//            }
//        }
//    }
//
//    if (n_coll < getNumEdges())
//    {
//        th1 = atan2(dy / dt, dx / dt) + successor->orientations[min_ind];
//        successor->theta = th1;
//        successor->steer_cost = (float) min_cost;
//        saveBestYaw(parent_node->x, parent_node->y, successor->x, successor->y, successor->theta);
//        saveBestCost(parent_node->x, parent_node->y, successor->x, successor->y, successor->steer_cost);
//
//        return true;
//    }
//
//    return false;
//}
//
//double POSQSteering::steerIntegrate(double x0, double y0, double th0,
//                            double x1, double y1, double th1)
//{
//    clearTraj();
//
//    // ROS_INFO("Steering %f %f %f,  %f %f %f ", x0, y0, th0, x1, y1, th1 );
//
//    double Kd = Kd_;
//    double Ktheta = Ktheta_;
//    double sl, sr, oldSl, oldSr, t, eot, dSl, dSr, dSm, dSd, vl, vr, enc_l, enc_r;
//    double cost = 0;
//    double b, dir;
//    double x_int, y_int, th_int, curr_x, curr_y, curr_th;
//    std::vector<double> steerstep_res;
//
//    enc_l = 0;
//    enc_r = 0;
//    sl = 0;
//    sr = 0;
//    oldSl = 0;
//    oldSr = 0;
//    eot = 0;
//    t = 0;
//
//    b = 0.40;
//    dir = 1;
//
//    x_int = x0;
//    y_int = y0;
//    th_int = th0;
//
//    double dt = 0.1;
//
//    while (eot == 0)
//    {
//        // calculate distance for both wheels
//        dSl = sl - oldSl;
//        dSr = sr - oldSr;
//        dSm = (dSl + dSr) / 2;
//        dSd = (dSr - dSl) / b;
//
//        curr_x = x_int + dSm * cos(th_int + dSd / 2);
//        curr_y = y_int + dSm * sin(th_int + dSd / 2);
//        curr_th = normangle(th_int + dSd, -M_PI);
//
//        steerstep_res = steer_step(curr_x, curr_y, curr_th, x1, y1, th1, t, b, (int) dir);
//
//        eot = steerstep_res[4];
//        vl = steerstep_res[0];
//        vr = steerstep_res[1];
//        //Increase the timer
//        t = t + dt;
//
//        // keep track of previous wheel position
//        oldSl = sl;
//        oldSr = sr;
//
//
//        // increase encoder values
//        enc_l = enc_l + dt * vl;
//        enc_r = enc_r + dt * vr;
//
//        sl = enc_l;
//        sr = enc_r;
//
//        if (COST_TYPE == 0)
//        {
//            cost = cost + Kd * sqrt((curr_x - x_int) * (curr_x - x_int) + (curr_y - y_int) * (curr_y - y_int)) +
//                   Ktheta * fabs(1 - cos(diff_angle_unwrap(curr_th, th_int)));
//        }
//
//
//        if (COST_TYPE == 2)
//        {
//            cost = cost + sqrt((curr_x - x_int) * (curr_x - x_int) + (curr_y - y_int) * (curr_y - y_int));
//        }
//
//        //save the state for the next sample
//        x_int = curr_x;
//        y_int = curr_y;
//        th_int = curr_th;
//
//        traj_x_.push_back(x_int);
//        traj_y_.push_back(y_int);
//        traj_th_.push_back(th_int);
//
//
//        if (environment_.collides(x_int, y_int))
//        {
//            /// Very high cost
//            cost = COLLISION_COST;
//            break;
//        }
//    }
//
//    if (COST_TYPE == 1 && cost != COLLISION_COST)
//        cost = CurvatureMetric::evaluateMetric(traj_x_, traj_y_);
//
//    return cost;
//}
//
//std::vector<double>
//POSQSteering::steer_step(double x_c, double y_c, double t_c, double x_end, double y_end, double t_end, double ct, double b,
//                 int dir)const
//{
//    /** This function will generate a vector of double as output:
//     *  [0] Vl velocity of the left wheel;
//     *  [1] Vr velocity of the right wheel;
//     *  [2] V translational Velocity;
//     *  [3] W Angular Velocity.
//     *  [4] EOT End Of Trajectory
//    **/
//    static double oldBeta;
//
//    double Krho, Kalpha, Kbeta, Vmax, RhoEndCondition, Kv;
//
//    Kv = 3.5; // 1
//    Krho = 1; // 1
//    Kalpha = 3; // 3
//    Kbeta = -1; // -1
//    Vmax = Krho;
//
//    /// The RRT* edges' length is related to the RhoEndCondition
//    // RhoEndCondition = 0.050;
//    // RhoEndCondition = ThetaStar::rho_end_condition;
//    RhoEndCondition = rho_end_condition;
//
//    if (ct == 0)
//    {
//        oldBeta = 0;
//    }
//
//    double dx, dy, rho, fRho, alpha, phi, beta, v, w, vl, vr, eot;
//    // rho
//    dx = x_end - x_c;
//    dy = y_end - y_c;
//    rho = sqrt(dx * dx + dy * dy);
//    fRho = rho;
//
//    if (fRho > (Vmax / Krho))
//    {
//        fRho = Vmax / Krho;
//    }
//
//    //alpha
//    alpha = (atan2(dy, dx) - t_c);
//    alpha = normangle(alpha, -M_PI);
//
//    //direction
//    if (dir == 0)
//    {
//        if (alpha > (M_PI / 2))
//        {
//            fRho = -fRho;
//            alpha = alpha - M_PI;
//        }
//        else if (alpha <= -M_PI / 2)
//        {
//            fRho = -fRho;
//            alpha = alpha + M_PI;
//        }
//    }
//    else if (dir == -1)
//    {
//        fRho = -fRho;
//        alpha = alpha + M_PI;
//        if (alpha > M_PI)
//        {
//            alpha = alpha - 2 * M_PI;
//        }
//    }
//
//    //phi
//    phi = t_end - t_c;
//    phi = normangle(phi, -M_PI);
//
//    beta = normangle((phi - alpha), -M_PI);
//
//    if ((abs(oldBeta - beta) > M_PI))
//    {
//        beta = oldBeta;
//    }
//    oldBeta = beta;
//
//    //set speed
//    v = Krho * tanh(Kv * fRho);
//    w = (Kalpha * alpha + Kbeta * beta);
//
//    if (fRho < RhoEndCondition)
//    {
//        eot = 1;
//        // ROS_INFO("END");
//        // exit(0);
//    }
//    else
//    {
//        eot = 0;
//    }
//
//    if (eot)
//    {
//        w = 0.;
//    }
//
//
//    //Convert speed to wheel speed
//    vl = v - w * b / 2;
//    if (abs(vl) > Vmax)
//    {
//        if (vl < 0)
//        {
//            vl = Vmax * -1;
//        }
//        else
//        {
//            vl = Vmax;
//        }
//    }
//
//    vr = v + w * b / 2;
//    if (abs(vr) > Vmax)
//    {
//        if (vr < 0)
//        {
//            vr = Vmax * -1;
//        }
//        else
//        {
//            vr = Vmax;
//        }
//    }
//
//    vector<double> res;
//    res.resize(5);
//    res[0] = vl;
//    res[1] = vr;
//    res[2] = v;
//    res[3] = w;
//    res[4] = eot;
//
//    return res;
//}
//
//bool POSQSteering::SteerVis(const GNode_base *parent_node, const GNode_base *succ, const GNode_base *pp_node, Trajectory *traj)
//{
//    // TODO come up with better test for start node...
////    if (parent_node->hasParent && parent_node->x != 3 && parent_node->y != 3)
////    {
////        ROS_INFO("Steer");
////        ROS_INFO("Parent %d %d", parent_node->x, parent_node->y);
////        ROS_INFO("GrandParent %d %d ", parent_node->parent->x, parent_node->parent->y);
////    }
//
//    traj->reset();
//
//    double x0, y0, th0, y1, x1, th1;
//    x0 = parent_node->x_r;
//    y0 = parent_node->y_r;
//    x1 = succ->x_r;
//    y1 = succ->y_r;
//
//    //TODO this changed (don't need getBestYaw())
//    /// Handle the first segment visualization
//    if (x0 == parent_node->start_x && y0 == parent_node->start_y)
//        th0 = parent_node->theta; //(double) problemDefinition_[4];
//    else
//        th0 = parent_node->theta; //getBestYaw(pp_node->x, pp_node->y, parent_node->x, parent_node->y);
//
//    //if (pp_node->y == parent_node->start_x && pp_node->y == parent_node->start_y)
//    //    th0 = parent_node->theta;
//
//    if (x0 == parent_node->start_x && y0 == parent_node->start_y)
//        th1 = succ->theta;
//    else
//        th1 = succ->theta; //getBestYaw((int) x0, (int) y0, (int) x1, (int) y1);
//
//    //ROS_INFO("Connecting %f %f %f to %f %f %f", x0, y0, th0, x1, y1, th1);
//
//    double dt = 0.1;
//
//
//    bool res = true;
//
//    double sl, sr, oldSl, oldSr, t, eot, dSl, dSr, dSm, dSd, vl, vr, enc_l, enc_r;
//    enc_l = 0;
//    enc_r = 0;
//    sl = 0;
//    sr = 0;
//    oldSl = 0;
//    oldSr = 0;
//    eot = 0;
//    t = 0;
//
//    double b, dir;
//    b = 0.30;
//    dir = 1;
//
//    double vv, ww;
//
//    double x_int, y_int, th_int, curr_x, curr_y, curr_th;
//    x_int = x0;
//    y_int = y0;
//    th_int = th0;
//    std::vector<double> steerstep_res;
//
//    while (eot == 0)
//    {
//        // calculate distance for both wheels
//        dSl = sl - oldSl;
//        dSr = sr - oldSr;
//        dSm = (dSl + dSr) / 2;
//
//        dSd = (dSr - dSl) / b;
//
//        curr_x = x_int + dSm * cos(th_int + dSd / 2);
//        curr_y = y_int + dSm * sin(th_int + dSd / 2);
//        curr_th = normangle(th_int + dSd, -M_PI);
//
//        steerstep_res = steer_step(curr_x, curr_y, curr_th, x1, y1, th1, t, b, (int) dir);
//        //Save the velocity commands,eot
//        vv = steerstep_res[2];
//        ww = steerstep_res[3];
//        eot = steerstep_res[4];
//        vl = steerstep_res[0];
//        vr = steerstep_res[1];
//
//        //Increase the timer
//        t = t + dt;
//
//        // keep track of previous wheel position
//        oldSl = sl;
//        oldSr = sr;
//
//
//        // increase encoder values
//        enc_l = enc_l + dt * vl;
//        enc_r = enc_r + dt * vr;
//
//        sl = enc_l;
//        sr = enc_r;
//
//        // ThetaStar::curr_solution_cost=ThetaStar::curr_solution_cost+Kv*sqrt((curr_x-x_int)*(curr_x-x_int)+(curr_y-y_int)*(curr_y-y_int)) + Ktheta*fabs(1-cos(diff_angle_unwrap(curr_th,th_int)));
//
//        //save the state for the next sample
//        x_int = curr_x;
//        y_int = curr_y;
//        th_int = curr_th;
//
//        // ThetaStar::old_trajectory_->addPointEnd(Tpoint(x_int, y_int, th_int));
//        // ThetaStar::old_trajectory_->addVelocities(vv,ww);
//
//        traj->addPointEnd(Tpoint(x_int, y_int, th_int));
//        traj->addVelocities(vv, ww);
//    }
//
//    //ROS_INFO("Connecting %f %f %f %f %f %f", x0, y0, th0, x1, y1, th1);
//
//    return res;
//}
//
//void POSQSteering::clearTraj()
//{
//    traj_th_.clear();
//    traj_x_.clear();
//    traj_y_.clear();
//}
//
//double POSQSteering::acceptance_probability(double old_cost, double new_cost, double temperature)
//{
//    if (new_cost < old_cost)
//        return 1.0;
//    else
//        return (exp((old_cost - new_cost) / temperature));
//}
//
//double POSQSteering::normangle(double a, double mina)
//{
//    double ap = a;
//    double minap = mina;
//    while (ap >= (minap + M_PI * 2.0))
//    {
//        ap -= 2.0 * M_PI;
//    }
//    while (ap < minap)
//    {
//        ap += 2.0 * M_PI;
//    }
//
//    return ap;
//}
//
//double POSQSteering::set_angle_to_range(double alpha, double min)
//{
//    while (alpha >= min + 2.0 * M_PI)
//    {
//        alpha -= 2.0 * M_PI;
//    }
//    while (alpha < min)
//    {
//        alpha += 2.0 * M_PI;
//    }
//    return alpha;
//}
//
//double POSQSteering::diff_angle_unwrap(double alpha1, double alpha2)
//{
//    // normalize angles alpha1 and alpha2
//    alpha1 = set_angle_to_range(alpha1, 0);
//    alpha2 = set_angle_to_range(alpha2, 0);
//
//    // take difference and unwrap
//    double delta = alpha1 - alpha2;
//    if (alpha1 > alpha2)
//    {
//        while (delta > M_PI)
//        {
//            delta -= 2.0 * M_PI;
//        }
//    }
//    else if (alpha2 > alpha1)
//    {
//        while (delta < -M_PI)
//        {
//            delta += 2.0 * M_PI;
//        }
//    }
//    return delta;
//}
//
//int POSQSteering::getNumEdges()
//{
//    return nEdges_;
//}
//
//int POSQSteering::saveBestCost(int xp, int yp, int xs, int ys, double cost)
//{
//    stringstream ss;
//    ss << xp << yp << xs << ys;
//    string key_new_element = ss.str();
//    best_cost_[key_new_element] = cost;
//    return 1;
//}
//
//double POSQSteering::getBestCost(int xp, int yp, int xs, int ys) const
//{
//    stringstream ss;
//    ss << xp << yp << xs << ys;
//    string key_look_for_element = ss.str();
//
//    if (best_cost_.find(key_look_for_element) == best_cost_.end())
//    {
//        return -1;
//    }
//    else
//    {
//        return best_cost_.find(key_look_for_element)->second;
//    }
//}
//
//int POSQSteering::saveBestYaw(int xp, int yp, int xs, int ys, double orient)
//{
//    stringstream ss;
//    ss << xp << yp << xs << ys;
//    string key_new_element = ss.str();
//    best_yaw_[key_new_element] = orient;
//    return 1;
//}
//
//double POSQSteering::getBestYaw(int xp, int yp, int xs, int ys)const
//{
//    stringstream ss;
//    ss << xp << yp << xs << ys;
//    string key_look_for_element = ss.str();
//
//    if (best_yaw_.find(key_look_for_element) == best_yaw_.end())
//    {
//        return -1;
//    }
//    else
//    {
//        return best_yaw_.find(key_look_for_element)->second;
//    }
//}
//
//int POSQSteering::saveOrientations(int xp, int yp, int xs, int ys, double *orient)
//{
//    stringstream ss;
//    ss << xp << yp << xs << ys;
//    string key_new_element = ss.str();
//    vector<double> values;
//
//    if (map_orientations_.find(key_new_element) == map_orientations_.end())
//    {
//        // ROS_INFO("Saving Orientation");
//
//        for (int i = 0; i < getNumEdges(); i++)
//        {
//            values.push_back(orient[i]);
//        }
//
//        map_orientations_[key_new_element] = values;
//
//        return 0;
//    }
//
//    // ROS_INFO("Key Already Existing!!!!");
//    return 1;
//}
//
//double *POSQSteering::getOrientations(int xp, int yp, int xs, int ys)
//{
//    stringstream ss;
//    ss << xp << yp << xs << ys;
//    string key_look_for_element = ss.str();
//
//    // ROS_INFO("Key key_look_for_element %s", key_look_for_element.c_str());
//
//    vector<double> res = map_orientations_[key_look_for_element];
//
//    double *r = new double[getNumEdges()];
//
//    for (int i = 0; i < getNumEdges(); i++)
//    {
//        r[i] = res[i];
//    }
//
//    return r;
//}
//
//bool POSQSteering::clearInternalData()
//{
//    try
//    {
//        this->best_yaw_.clear();
//        this->best_cost_.clear();
//        this->map_orientations_.clear();
//        this->traj_x_.clear();
//        this->traj_y_.clear();
//        this->traj_th_.clear();
//    }
//    catch (int e)
//    {
//        ROS_INFO("An exception occurred. Exception Nr. %d", e);
//        return false;
//    }
//
//    return true;
//}

POSQSteering::POSQSteering()
{
//    _posq.DT = 0.2;
}

bool POSQSteering::Steer(GNode_base *parent_node, GNode_base *successor)
{
    // TODO check for collision?
    return true;
}

bool POSQSteering::Steer(const GNode_base *parent_node, const GNode_base *succ, Trajectory *traj)
{
    auto *start = _space.allocState()->as<ob::SE2StateSpace::StateType>();
    auto *goal = _space.allocState()->as<ob::SE2StateSpace::StateType>();
    start->setX(parent_node->x_r);
    start->setY(parent_node->y_r);
    start->setYaw(parent_node->theta);
    goal->setX(succ->x_r);
    goal->setY(succ->y_r);
    goal->setYaw(succ->theta);
    double distance;
    std::vector<UnicycleState> states = _posq.steer(start, goal, distance);
    delete start;
    delete goal;
    if (states.size() > 3)
    {
        // TODO THIS IS A HACK!!!
//            states.erase(states.begin());
//            states.erase(states.begin());
//        states.pop_back();
//        states.pop_back();

//            x = x + dSm * cos(th + dSd / 2);
//            y = y + dSm * sin(th + dSd / 2);
//            th = normAngle(th + dSd, -M_PI);
//        double x = 0.5*(succ->x_r+states.back().x_);
//        double y = 0.5*(succ->y_r+states.back().y_);
//        double th = 0.5*(succ->theta+states.back().yaw_);
//        states.push_back(UnicycleState(x, y, th));
    }
    for (auto &s : states)
    {
        traj->addPointEnd(Tpoint(s.x_, s.y_, s.yaw_));
        traj->addVelocities(1, 1);
    }

    // TODO check for collision?
    return true;
}
