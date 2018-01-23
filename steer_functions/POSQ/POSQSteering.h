#pragma once

#include "steer_functions/steer_base.h"
#include "POSQ.hpp"


class POSQSteering : public Steer_base
{
public:
    POSQSteering();

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

//    typedef std::map<std::string, std::vector<double> > Map;
//    typedef std::map<std::string, double> MapYaw;
//    typedef std::map<std::string, double> MapCost;
//
//private:
//    Map map_orientations_;
//    MapYaw best_yaw_;
//    MapCost best_cost_;
//
//    int nEdges_;
//
//    int *problemDefinition_;
//
//    double Kd_;
//    double Ktheta_;
//
//    std::vector<double> traj_x_;
//    std::vector<double> traj_y_;
//    std::vector<double> traj_th_;
//
//    Environment environment_;
//
//public:
//    static double minYaw;
//    static double maxYaw;
//
//    static double rho_end_condition;
//
//    static int TYPE;
//    static int COST_TYPE;
//
//public:
//    POSQSteering(int *prbdef,
//         const Environment &environment,
//         double kd = PlannerSettings::Kv,
//         double ktheta = PlannerSettings::Ktheta,
//         double minYaw = -0.7854,
//         double maxYaw = 0.7854,
//         int nEdges = PlannerSettings::numberEdges,
//         double rho = PlannerSettings::rhoEndcondition,
//         int type = 0,
//         int COST_TYPE = 0);
//
//    bool Steer(GNode_base *parent_node, GNode_base *successor) override;
//    bool SteerRandomFeasible(GNode_base *parent_node, GNode_base *successor);
//    bool SteerDiscreteOptimal(GNode_base *parent_node, GNode_base *successor);
//    bool SteerGridAnglesUSC(GNode_base *parent_node, GNode_base *successor);
//    bool SteerAnnealing(GNode_base *parent_node, GNode_base *successor);
//    bool SteerStraight(GNode_base *parent_node, GNode_base *successor);
//    bool SteerGridAngles(GNode_base *parent_node, GNode_base *successor);
//    bool SteerRandomAngles(GNode_base *parent_node, GNode_base *successor);
//
//    bool SteeringRewire(GNode_base *parent_node, GNode_base *successor);
//    double steerIntegrate(double x0, double y0, double th0, double x1, double y1, double th1);
//
//    /**
//     * Single Step of the steering function, (x_c,y_c,t_c) current pose of the robot, (x_end,y_end,t_end)
//     * final pose of the robot, ct current time step, b size of the robot, dir direction of the robot.
//     */
//    std::vector<double> steer_step(double x_c, double y_c, double t_c,
//                                   double x_end, double y_end, double t_end,
//                                   double ct, double b, int dir)const;
//
//
//    static double normangle(double a, double mina);
//    static double set_angle_to_range(double alpha, double min);
//    static double diff_angle_unwrap(double alpha1, double alpha2);
//
//
//    /**
//     * Generate the trajectory to connect the parent_node to the current node. Used to generate
//     * the trajectory to visualize.
//     */
//    bool SteerVis(const GNode_base *parent_node, const GNode_base *succ, const GNode_base *pp_node, Trajectory *traj) override;
//
//
//    double acceptance_probability(double old_cost, double new_cost, double temperature);
//
//    void clearTraj();
//
//    /**
//     * Return the number of sampled Edges.
//     */
//    int getNumEdges();
//
//
//    /**
//     * Save the best cost associated to the key.
//     */
//    int saveBestCost(int xp, int yp, int xs, int ys, double cost);
//
//    /**
//     * Find the best cost associated to the key.
//     */
//    double getBestCost(int xp, int yp, int xs, int ys) const;
//
//
//    /**
//     * Save the best Yaw associated to the key.
//     */
//    int saveBestYaw(int xp, int yp, int xs, int ys, double orient);
//
//    /**
//     * Get the Yaw angle associated to the key.
//     */
//    double getBestYaw(int xp, int yp, int xs, int ys)const;
//
//
//    int saveOrientations(int xp, int yp, int xs, int ys, double *orient);
//
//    double *getOrientations(int xp, int yp, int xs, int ys);
//
//    /**
//     * Clear internal states of the POSQ function.
//     */
//    bool clearInternalData();

};
