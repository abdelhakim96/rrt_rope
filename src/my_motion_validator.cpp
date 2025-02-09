#include "my_motion_validator.hpp"

CustomMotionValidator::CustomMotionValidator(const ompl::base::SpaceInformationPtr &si) : ompl::base::MotionValidator(si) {}

bool CustomMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const
{
    // Check if the start and end states are valid
    if (!si_->isValid(s1) || !si_->isValid(s2))
        return false;

    // Interpolate between the states and check each intermediate state for validity
    int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
    auto *interpolatedState = si_->allocState();

    for (int i = 1; i < nd; ++i)
    {
        si_->getStateSpace()->interpolate(s1, s2, (double)i / (double)nd, interpolatedState);
        if (!si_->isValid(interpolatedState))
        {
            si_->freeState(interpolatedState);
            return false;
        }
    }

    si_->freeState(interpolatedState);
    return true;
}

bool CustomMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const
{
    // Check if the start state is valid
    if (!si_->isValid(s1))
    {
        lastValid.first = si_->cloneState(s1);
        lastValid.second = 0.0;
        return false;
    }

    // Interpolate between the states and check each intermediate state for validity
    int nd = si_->getStateSpace()->validSegmentCount(s1, s2);
    auto *interpolatedState = si_->allocState();

    for (int i = 1; i < nd; ++i)
    {
        si_->getStateSpace()->interpolate(s1, s2, (double)i / (double)nd, interpolatedState);
        if (!si_->isValid(interpolatedState))
        {
            lastValid.first = si_->cloneState(interpolatedState);
            lastValid.second = (double)(i - 1) / (double)nd;
            si_->freeState(interpolatedState);
            return false;
        }
    }

    si_->freeState(interpolatedState);
    lastValid.second = 1.0;
    return true;
}