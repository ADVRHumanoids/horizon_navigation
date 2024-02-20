#include <obstacle.h>

CasadiObstacle::CasadiObstacle(Eigen::Vector3d origin, Eigen::Vector3d radius):
    _origin(origin),
    _radius(radius)
{
//    _distance =
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
    casadi_int dim = 2;
    auto pos = casadi::SX::sym("pos", dim);
    auto origin = casadi::SX::sym("origin", dim);
    auto radius = casadi::SX::sym("radius", dim);

    auto f = casadi::SX::sumsqr(pos - origin) - casadi::SX::pow(radius, 2);

    casadi::Function ObstacleFun("simple_obstacle", {pos, origin, radius}, {f}, {"pos", "origin", "radius"}, {"obstacle"});

    return ObstacleFun;

}

Eigen::Vector3d CasadiObstacle::getOrigin()
{
    return _origin;
}

//double CasadiObstacle::getDistance()
//{
//    return _distance;
//}

Eigen::Vector3d CasadiObstacle::getRadius()
{
    return _radius;
}
