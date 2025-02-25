#include "collision_checker.hpp"
#include <nvblox/mesh/mesh.h>
#include <nvblox/nvblox.h>
#include <random>


CollisionChecker::CollisionChecker(nvblox::Mapper& input_mapper)
    : mapper(input_mapper)
    , bb(nvblox::getAABBOfAllocatedBlocks(mapper.esdf_layer()))
    , step(mapper.voxel_size_m() * 3.0)
{
}

bool CollisionChecker::isCollisionFree(const Eigen::Vector3d& point, double eps)
{
    auto r = mapper.esdf_layer().getVoxel(point.cast<float>());

    bool is_present = r.second;
    if (!is_present)
        return  true;
    nvblox::EsdfVoxel voxel = r.first;

    return !r.first.is_inside && voxel.squared_distance_vox > eps;
}

Eigen::Vector3d CollisionChecker::getRandomPoint()
{
    return bb.sample().cast<double>();
}

Eigen::Vector3d CollisionChecker::getRandomPointFree()
{
    Eigen::Vector3d point = getRandomPoint();
    while (!isCollisionFree(point))
    {
        point = getRandomPoint();
    }
    return point;
}