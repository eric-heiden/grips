#include "POSQStateSpace.h"
#include <ompl/base/SpaceInformation.h>
#include <ompl/util/Exception.h>
#include <queue>
#include <boost/math/constants/constants.hpp>


void POSQStateSpace::initialize()
{
    state_space_ = new POSQ();
}

double POSQStateSpace::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
    double distance = state_space_->distance(state1, state2);
//    OMPL_DEBUG("distance %f \n", distance);
    return distance;
}


void POSQStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
                                 ompl::base::State *state) const
{
    bool firstTime = true;
    vector<UnicycleState> states;
    states.clear();
    interpolate(from, to, t, firstTime, states, state);
}

void POSQStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, const double t,
                                 bool &firstTime, vector<UnicycleState> &states, ompl::base::State *state) const
{
    if (firstTime)
    {
        if (t >= 1.)
        {
            if (to != state)
                copyState(state, to);
            return;
        }
        if (t <= 0.)
        {
            if (from != state)
                copyState(state, from);
            return;
        }

//	    OMPL_DEBUG("Generating states.. ");
      double distance = 0;
	    states = state_space_->steer(from, to, distance);
//		  OMPL_DEBUG("state number %d ", (int)states.size());

      firstTime = false;
    }

//    OMPL_DEBUG("Interpolating again.. ");
    interpolate(from, states, t, state);
}

void POSQStateSpace::interpolate(const ompl::base::State *from, const vector<UnicycleState> states, double t,
                                 ompl::base::State *state) const
{
    std::vector<UnicycleState> v(states);
	UnicycleState result = state_space_->interpolate(from, v, t);
    state->as<StateType>()->setX(result.x_);
    state->as<StateType>()->setY(result.y_);
    state->as<StateType>()->setYaw(result.yaw_);
//    OMPL_DEBUG("Time: %f, x: %f  y:%f  z: %f ", t, result.x_, result.y_, result.yaw_);
}

void POSQMotionValidator::defaultSettings()
{
    stateSpace_ = dynamic_cast<POSQStateSpace *>(si_->getStateSpace().get());
    if (stateSpace_ == nullptr)
        throw ompl::Exception("No state space for motion validator");
}

bool POSQMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2,
                                          std::pair<ompl::base::State *, double> &lastValid) const
{
    /* assume motion starts in a valid configuration so s1 is valid */

    bool result = true, firstTime = true;
    vector<UnicycleState> states;
    int nd = stateSpace_->validSegmentCount(s1, s2);

    if (nd > 1)
    {
         /* temporary storage for the checked state */
        ompl::base::State *test = si_->allocState();

        for (int j = 1; j < nd; ++j)
        {
            stateSpace_->interpolate(s1, s2, (double)j / (double)nd, firstTime, states, test);
            if (!si_->isValid(test))
            {
                lastValid.second = (double)(j - 1) / (double)nd;
                if (lastValid.first != nullptr)
                    stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, states, lastValid.first);
                result = false;
                break;
            }
        }
        si_->freeState(test);
    }

    if (result)
        if (!si_->isValid(s2))
        {
            lastValid.second = (double)(nd - 1) / (double)nd;
            if (lastValid.first != nullptr)
                stateSpace_->interpolate(s1, s2, lastValid.second, firstTime, states, lastValid.first);
            result = false;
        }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}

bool POSQMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    /* assume motion starts in a valid configuration so s1 is valid */
    if (!si_->isValid(s2))
        return false;

    bool result = true, firstTime = true;

    std::vector<UnicycleState> states;
    states.clear();

    int nd = stateSpace_->validSegmentCount(s1, s2);

    // /* initialize the queue of test positions */
    std::queue<std::pair<int, int>> pos;
    if (nd >= 2)
    {
        pos.push(std::make_pair(1, nd - 1));

         /* temporary storage for the checked state */
        ompl::base::State *test = si_->allocState();

        /* repeatedly subdivide the path segment in the middle (and check the middle) */
        while (!pos.empty())
        {
            std::pair<int, int> x = pos.front();

            int mid = (x.first + x.second) / 2;
            stateSpace_->interpolate(s1, s2, (double)mid / (double)nd, firstTime, states, test);

            if (!si_->isValid(test))
            {
                result = false;
                break;
            }

            pos.pop();

            if (x.first < mid)
                pos.push(std::make_pair(x.first, mid - 1));
            if (x.second > mid)
                pos.push(std::make_pair(mid + 1, x.second));
        }

        si_->freeState(test);
    }

    if (result)
        valid_++;
    else
        invalid_++;

    return result;
}
