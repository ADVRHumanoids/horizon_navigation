#include <obstacle.h>

SphereObstacle::SphereObstacle(Eigen::Vector3d origin, Eigen::Vector3d radius):
    _origin(origin),
    _radius(radius)
{
    _angle = atan2(_origin[1], _origin[0]);
}

Eigen::Vector3d SphereObstacle::getOrigin()
{
    return _origin;
}

Eigen::Vector3d SphereObstacle::getRadius()
{
    return _radius;
}

double SphereObstacle::getAngle()
{
    return _angle;
}

CasadiObstacle::CasadiObstacle()
{
}

casadi::Function CasadiObstacle::gaussFormulation()
{
    casadi_int dim = 2;
    auto pos = casadi::SX::sym("pos", dim);
    auto origin = casadi::SX::sym("origin", dim);
    auto sigma = casadi::SX::sym("sigma", dim);
    auto amplitude = casadi::SX::sym("amplitude", 1);

    // simple gaussian curve
    auto fx = casadi::SX::pow(pos(0) - origin(0), 2) / (2 * casadi::SX::pow(sigma(0), 2));
    auto fy = casadi::SX::pow(pos(1) - origin(1), 2) / (2 * casadi::SX::pow(sigma(1), 2));

    auto f = amplitude * (- (fx + fy));

    casadi::Function ObstacleFun("gauss_obstacle", {pos, origin, sigma, amplitude}, {f}, {"pos", "origin", "sigma", "amplitude"}, {"obstacle"});

    return ObstacleFun;

}

casadi::Function CasadiObstacle::simpleFormulation()
{
    auto robot_pos = casadi::SX::sym("robot_pos", 2);
    auto obstacle_origin = casadi::SX::sym("obstacle_origin", 2);
    auto obstacle_radius = casadi::SX::sym("obstacle_radius", 1);
    auto robot_radius = casadi::SX::sym("robot_radius", 1);

    // auto f = casadi::SX::sumsqr(robot_pos - obstacle_origin) - casadi::SX::pow(obstacle_radius + robot_radius, 2);
    auto center_distances = casadi::SX::sqrt(0.0001 + casadi::SX::sumsqr(robot_pos - obstacle_origin));

    auto f = center_distances - (obstacle_radius + robot_radius);

    casadi::Function ObstacleFun("simple_obstacle", {robot_pos, obstacle_origin, obstacle_radius, robot_radius}, {f}, {"robot_pos", "obstacle_origin", "obstacle_radius", "robot_radius"}, {"obstacle"});

    return ObstacleFun;

}

//Eigen::Vector3d CasadiObstacle::getOrigin()
//{
//    return _origin;
//}

//Eigen::Vector3d CasadiObstacle::getRadius()
//{
//    return _radius;
//}
