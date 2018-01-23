/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2010, Rice University
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
*   * Neither the name of the Rice University nor the names of its
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

#pragma once

/* Author: Mark Moll */

// Adapted by Luigi Palmieri CR/AER to combine it with POSQSteering

#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/MotionValidator.h>
#include <boost/math/constants/constants.hpp>

#include "POSQ.hpp"

/** \brief POSQ is a class to have the POSQ function as OMPL state space.
*/

class POSQStateSpace : public ompl::base::SE2StateSpace
{
public:

    POSQStateSpace()
    {
        initialize();
    }

    ~POSQStateSpace() override
    {
        delete state_space_;
    }

    /** \brief Initialize the requested steering function */
    void initialize();


    /** \brief Compute the distance between the two states */
    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const override;


    /** \brief Interpolate between two states from and to. Returns the state, of the curve connecting the states (from, to) computed at time t.*/
    void interpolate(const ompl::base::State *from, const ompl::base::State *to,
                     double t, ompl::base::State *state) const override;

    virtual void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, bool &firstTime,
                             vector<UnicycleState> &states, ompl::base::State *state) const;

    virtual void interpolate(const ompl::base::State *from, const vector<UnicycleState> states, double t,
                             ompl::base::State *state) const;


    /** \brief Checks if the state was properly initialized */
    void sanityChecks() const override
    {
        std::cout << "sanity check!!!!" << std::endl;
        double zero = std::numeric_limits<double>::epsilon();
        double eps = .1;  // rarely such a large error will occur
        ompl::base::StateSpace::sanityChecks(zero, eps, ~STATESPACE_INTERPOLATION);
    }


protected:

    // virtual void interpolate(const State *from, double t, State *state) const;


    /** \brief Object from the steering function workspace*/
    POSQ *state_space_;
};


class POSQMotionValidator : public ompl::base::MotionValidator
{
public:
    POSQMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::MotionValidator(si)
    {
        defaultSettings();
    }

    ~POSQMotionValidator() override = default;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

private:
    POSQStateSpace *stateSpace_;
    void defaultSettings();
};
