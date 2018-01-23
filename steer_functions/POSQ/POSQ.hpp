/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, Social Robotics Laboratory, University of Freiburg.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Luigi Palmieri */

#pragma once
#include <ompl/base/spaces/SE2StateSpace.h>

#include "base/PlannerSettings.h"


using namespace std;
/** \brief Definition of the namespaces */

namespace ob = ompl::base;

/** \brief Internal definition of the Unicycle state type */
struct UnicycleState
{
    // pose
    double x_;
    double y_;
    double yaw_;

    UnicycleState()
    {
        x_ = 0.0;
        y_ = 0.0;
        yaw_ = 0.0;
    }

    UnicycleState(double xx, double yy, double zz) : x_(xx), y_(yy), yaw_(zz)
    {
    }

    UnicycleState(double *p)
    {
        x_ = p[0];
        y_ = p[1];
        yaw_ = p[2];
    }

    UnicycleState &operator=(UnicycleState const &copy)
    {
        x_ = copy.x_;
        y_ = copy.y_;
        yaw_ = copy.yaw_;
        return *this;
    }

    UnicycleState(const UnicycleState &copy)
    {
        x_ = copy.x_;
        y_ = copy.y_;
        yaw_ = copy.yaw_;
    }

    ~UnicycleState(void)
    {
    }
};

/** \brief Internal definition of the Unicycle control type*/
struct UnicycleControl
{
    // v, translational velocity
    double v_;
    // w, rotational velocity
    double w_;

    UnicycleControl(double vv = 0, double ww = 0) : v_(vv), w_(ww)
    {}

    UnicycleControl(const double* p)
    {
        v_ = p[0];
        w_ = p[1];
    }

    UnicycleControl& operator=(UnicycleControl const& copy) = default;

    UnicycleControl(const UnicycleControl& copy)
    {
        v_ = copy.v_;
        w_ = copy.w_;
    }

    virtual ~UnicycleControl() = default;
};

/** \brief POSQ class that implements the POSQ steering function */
class POSQ
{
public:

/** \brief length of the wheels axis */
    const double B = .54;

/** \brief Integration Time Step  */
    double DT{0.1};

/** \brief Max Size of the Control (HacK: in this case equal to the dimension) */
    const int MAX_SIZE = 10001;

/** \brief Value used to mark the end of the vector, no used at the moment */
    const double END_CONTROLS = 2000;


    /** \brief Internal Results of the integration */
    double *result_;
    double *intRes_;

    /** \brief Internal Counter to access the current control to propagate */
    int *ind_;

    double Krho, Kalpha, Kbeta, Kv, Vmax, RhoEndCondition;

    ompl::base::StateSpacePtr space_;

    POSQ() : DT(0.1)
    {
        space_ = std::make_shared<ompl::base::SE2StateSpace>();
        result_ = (double *) malloc(sizeof(double) * 5);
        intRes_ = (double *) malloc(sizeof(double) * 5);
        ind_ = (int *) malloc(sizeof(int) * 1);


//        Kalpha = 2.5; //6.91;
//        Kbeta = -3;
//        /** Read the paper and see how to set the proper gain values
//
//           double Kphi;
//           Kphi    = -1;
//        **/
//        Krho = 25.2; //15.5; //0.2;
//        Kv = 0.2; //13.8;// .2; //0.01; //PlannerSettings::Kv;
//
//        Vmax = Krho;
//        RhoEndCondition = 0.015; //PlannerSettings::rhoEndcondition;

        Kalpha = 6.91;
        Kbeta = -1;
        /** Read the paper and see how to set the proper gain values

           double Kphi;
           Kphi    = -1;
        **/
        Krho = 2; //15.5; //0.2;
        Kv = 1.2; //13.8;// .2; //0.01; //PlannerSettings::Kv;

        Vmax = Krho;
        RhoEndCondition = 0.1; //PlannerSettings::rhoEndcondition;

        //////////////////////////////////////////////////////////////////////////////////
        // Luigi's values:
        //////////////////////////////////////////////////////////////////////////////////
        Kalpha  = 3;
        Kbeta   = -1;
        /** Read the paper and see how to set the proper gain values

           double Kphi;
           Kphi    = -1;
        **/
        Krho = 1.2;
        Kv   = 1;
        Vmax = 2000; //Krho;
    }

    /** \brief Set the current result of the integration */
    void setRes(const double *r) const
    {
        this->intRes_[0] = r[0];
        this->intRes_[1] = r[1];
        this->intRes_[2] = r[2];
        this->intRes_[3] = r[3];
        this->intRes_[4] = r[4];
    }


    /** \brief Normalize the angle a rispect to the minimum angle mina.
        \param double a
        \param double mina
    */
    double normAngle(double a, double mina) const
    {
        double ap, minap;
        ap = a;
        minap = mina;

        while (ap >= (minap + M_PI * 2))
        {
            ap = ap - M_PI * 2;
        }

        while (ap < minap)
        {
            ap = ap + M_PI * 2;
        }

        return ap;
    }

    /** \brief Set the angle a in the range [min, min+2*M_PI].
        \param double alpha
        \param double min
    */
    double setAngleToRange(double alpha, double min) const
    {
        while (alpha >= min + 2.0 * M_PI)
        {
            alpha -= 2.0 * M_PI;
        }

        while (alpha < min)
        {
            alpha += 2.0 * M_PI;
        }

        return alpha;
    }

    /** \brief Single step of the Steer Function.
        \param double x_c, initial x-coord
        \param double y_c, initial y-coord
        \param double t_c, initial orientation
        \param double x_end, final x-coord.
        \param double y_end, final y-coord.
        \param double t_end, final orientation
        \param double c_t, integration time step
        \param double b, length of the wheel axis
        \param double dir, direction of the robot dir==1 forward
    */
    double *posControlStep(double x_c, double y_c, double t_c,
                           double x_end, double y_end, double t_end, double ct, double b, int dir) const
    {
        /** This function will generate a vector of double as output:
        *  [0] Vl velocity of the left wheel;
        *  [1] Vr velocity of the right wheel;
        *  [2] V translational Velocity;
        *  [3] W Angular Velocity.
        *  [4] EOT End Of Trajectory
        **/
        static double oldBeta;
//
//        double Krho, Kalpha, Kbeta, Kv, Vmax, RhoEndCondition;
//
//        Kalpha = 18; //6.91;
//        Kbeta = -5;
//        /** Read the paper and see how to set the proper gain values
//
//           double Kphi;
//           Kphi    = -1;
//        **/
//        Krho = 0.2; //15.5; //0.2;
//        Kv = 0.2; //13.8;// .2; //0.01; //PlannerSettings::Kv;
//
//        Vmax = Krho;
//        RhoEndCondition = 0.15; //PlannerSettings::rhoEndcondition;

        if (Kalpha+Kbeta-Krho*Kv <= 0)
            std::cerr << "K_alpha + K_phi - K_rho K_v = " << Kalpha+Kbeta-Krho*Kv << std::endl;
        if (Kalpha+2*Kbeta-2/M_PI*Krho*Kv <= 0)
            std::cerr << "K_alpha + 2 K_phi - 2/pi K_rho K_v = " << Kalpha+2*Kbeta-2/M_PI*Krho*Kv << std::endl;

        if (ct == 0)
            oldBeta = 0;

        double dx, dy, rho, fRho, alpha, phi, beta, v, w, vl, vr;

        // rho
        bool eot = true;
        dx = x_end - x_c;
        dy = y_end - y_c;
        rho = sqrt(dx * dx + dy * dy);
        fRho = rho;

        if (fRho > (Vmax / Krho))
            fRho = Vmax / Krho;

        // alpha
        alpha = atan2(dy, dx) - t_c;
        alpha = normAngle(alpha, -M_PI);

        // direction
        if (dir == 0)
        {
            if (alpha > (M_PI / 2))
            {
                fRho = -fRho;
                alpha = alpha - M_PI;
            }
            else if (alpha <= -M_PI / 2)
            {
                fRho = -fRho;
                alpha = alpha + M_PI;
            }
        }
        else if (dir == -1)
        {
            fRho = -fRho;
            alpha = alpha + M_PI;

            if (alpha > M_PI)
            {
                alpha = alpha - 2 * M_PI;
            }
        }

        // phi
        phi = t_end - t_c;
        phi = normAngle(phi, -M_PI);
        beta = normAngle(phi - alpha, -M_PI);

        if ((abs(oldBeta - beta) > M_PI))
            beta = oldBeta;

        oldBeta = beta;

        //set speed
        v = Krho * tanh(Kv * fRho);
        w = Kalpha * alpha + Kbeta * beta;

        eot = rho < RhoEndCondition;

        if (eot)
            w = 0.;

        //Convert speed to wheel speed
        vl = v - w * b / 2;
        if (abs(vl) > Vmax)
        {
            if (vl < 0)
                vl = Vmax * -1;
            else
                vl = Vmax;
        }

        vr = v + w * b / 2;
        if (abs(vr) > Vmax)
        {
            if (vr < 0)
                vr = Vmax * -1;
            else
                vr = Vmax;
        }

        result_[0] = vl;
        result_[1] = vr;
        result_[2] = (vl + vr) / 2;
        result_[3] = (vr - vl) / b;
        result_[4] = (int)eot;

        return result_;
    }

    /** \brief Propagate the model of the system forward, starting at a given state, with a given control, for a given number of steps.
        \param state from Initial state of the steering
        \param state to Ending state of the steering
        \param double duration, number of control steps needed to steer the robot
    */

    std::vector<UnicycleState> steer(const ob::State *from, const ob::State *to, double &distance) const
    {
        double sl, sr, oldSl, oldSr, t, eot, dSl, dSr, dSm, dSd, vl, vr, enc_l, enc_r, dir;
        dir = 1;
        enc_l = 0;
        enc_r = 0;
        sl = 0;
        sr = 0;
        oldSl = 0;
        oldSr = 0;
        eot = 0;
        t = 0;
        vl = 0;
        vr = 0;
        distance = 0;
        double x, y, th;
        double x_fin, y_fin, th_fin;

        x = from->as<ob::SE2StateSpace::StateType>()->getX();
        y = from->as<ob::SE2StateSpace::StateType>()->getY();
        th = from->as<ob::SE2StateSpace::StateType>()->getYaw();

        x_fin = to->as<ob::SE2StateSpace::StateType>()->getX();
        y_fin = to->as<ob::SE2StateSpace::StateType>()->getY();
        th_fin = to->as<ob::SE2StateSpace::StateType>()->getYaw();

        double vv, ww;
        vv = 0;
        ww = 0;

        std::vector<UnicycleControl> controls;
        std::vector<UnicycleState> states;
        controls.clear();
        states.clear();

        int n_steps;
        n_steps = 0;
        while (eot == 0)
        {
            // calculate distance for both wheels
            dSl = sl - oldSl;
            dSr = sr - oldSr;
            dSm = (dSl + dSr) / 2;
            dSd = (dSr - dSl) / B;
            x = x + dSm * cos(th + dSd / 2);
            y = y + dSm * sin(th + dSd / 2);
            th = normAngle(th + dSd, -M_PI);
            // intRes= posControlStep (x,y,th,x_fin,y_fin,th_fin, t,b,dir);
            setRes(posControlStep(x, y, th, x_fin, y_fin, th_fin, t, B, dir));
            //Save the velocity commands,eot
            vv = intRes_[2];
            ww = intRes_[3];
            eot = intRes_[4];
            vl = intRes_[0];
            vr = intRes_[1];
            //Increase the timer
            t = t + DT;
            // Count the number of steps
            n_steps++;
            // keep track of previous wheel position
            oldSl = sl;
            oldSr = sr;
            // increase encoder values
            enc_l = enc_l + DT * vl;
            enc_r = enc_r + DT * vr;
            sl = enc_l;
            sr = enc_r;
            if (eot == 1)
            {
                // Add current values to the Trajectory
                states.push_back(UnicycleState(x, y, th));
                controls.push_back(UnicycleControl(vv, ww));

                // save the last state!!!
                double xf, yf, yawf, vf, wf;

                vf = intRes_[2];
                wf = intRes_[3];
                dSl = sl - oldSl;
                dSr = sr - oldSr;
                dSm = (dSl + dSr) / 2;
                dSd = (dSr - dSl) / B;
                double dx = dSm * cos(th + dSd / 2);
                double dy = dSm * sin(th + dSd / 2);
                xf = x + dx;
                yf = y + dy;
                yawf = normAngle(th + dSd, -M_PI);
                distance += sqrt(dx * dx + dy * dy);
                states.push_back(UnicycleState(xf, yf, yawf));
                controls.push_back(UnicycleControl(vf, wf));
            }
            else
            {
                // Add current values to the Trajectory
                UnicycleState s;
                s.x_ = x;
                s.y_ = y;
                s.yaw_ = th;
                states.push_back(s);

                UnicycleControl c;
                c.v_ = vv;
                c.w_ = ww;
                controls.push_back(c);
            }
        }

//        if (states.size() > 3)
//        {
//            // TODO THIS IS A HACK!!!
////            states.erase(states.begin());
////            states.erase(states.begin());
//            states.pop_back();
//            states.pop_back();
//
////            x = x + dSm * cos(th + dSd / 2);
////            y = y + dSm * sin(th + dSd / 2);
////            th = normAngle(th + dSd, -M_PI);
//            x = 0.5*(states.back().x_+x_fin);
//            y = 0.5*(states.back().y_+y_fin);
//            th = normAngle(0.5*(states.back().yaw_+th_fin), -M_PI);
//            states.push_back(UnicycleState(x, y, th));
//        }
        return states;
    }

    /** \brief Compute the distance between two states */
    double distance(const ompl::base::State *state1, const ompl::base::State *state2)
    {
        double distance = 0;
        steer(state1, state2, distance);
        return distance;
    }

    /** \brief Return state at time t */
    UnicycleState interpolate(const ompl::base::State *from, std::vector<UnicycleState> states,
                              const double t)
    {
        // returning the state at time t;
        int n_states = states.size();
        int index_state_at_t_time = int(double(n_states) * t);
        return states[index_state_at_t_time];
    }
};
