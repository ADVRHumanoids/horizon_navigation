#include <obstacles_generator.h>

ObstacleGenerator::ObstacleGenerator()
{
}

casadi::Function ObstacleGenerator::gaussObstacle()
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

    casadi::Function Obstacle("gauss_obstacle", {pos, origin, sigma, amplitude}, {f}, {"pos", "origin", "sigma", "amplitude"}, {"obstacle"});

    return Obstacle;

}

casadi::Function ObstacleGenerator::simpleObstacle()
{
    casadi_int dim = 2;
    auto pos = casadi::SX::sym("pos", dim);
    auto origin = casadi::SX::sym("origin", dim);
    auto radius = casadi::SX::sym("radius", dim);

    auto f = casadi::SX::sumsqr(pos - origin) - casadi::SX::pow(radius, 2);

    casadi::Function Obstacle("simple_obstacle", {pos, origin, radius}, {f}, {"pos", "origin", "radius"}, {"obstacle"});

    return Obstacle;

}


