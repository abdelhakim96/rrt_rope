#pragma once 
#include "range.hpp"
#include "nvblox/core/types.h"
#include "nvblox/mapper/mapper.h"

class CollisionChecker
{
public:
    nvblox::Mapper& mapper;
    Eigen::AlignedBox3f bb;
    double step;

    CollisionChecker(nvblox::Mapper& input_mapper);
public:
    bool isCollisionFree(const Eigen::Vector3d& point, double eps =10);
    Eigen::Vector3d getRandomPoint();
    Eigen::Vector3d getRandomPointFree();
};