#ifndef CUSTOM_MOTION_VALIDATOR_HPP
#define CUSTOM_MOTION_VALIDATOR_HPP

#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <utility>

class CustomMotionValidator : public ompl::base::MotionValidator
{
public:
    CustomMotionValidator(const ompl::base::SpaceInformationPtr &si);

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;
};

#endif // CUSTOM_MOTION_VALIDATOR_HPP