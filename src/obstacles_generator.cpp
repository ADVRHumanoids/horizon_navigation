#include <obstacles_generator.h>

ObstacleGenerator::ObstacleGenerator()
{
}

casadi::Function ObstacleGenerator::gaussObstacle()
{
    casadi_int dim = 2;
    auto _pos = casadi::SX::sym("pos", dim);
    auto _origin = casadi::SX::sym("origin", dim);
    auto _sigma = casadi::SX::sym("sigma", dim);
    auto _amplitude = casadi::SX::sym("amplitude", 1);

    // simple gaussian curve
    auto fx = casadi::SX::pow(_pos(0) - _origin(0), 2) / (2 * casadi::SX::pow(_sigma(0), 2));
    auto fy = casadi::SX::pow(_pos(1) - _origin(1), 2) / (2 * casadi::SX::pow(_sigma(1), 2));

    auto f = _amplitude * (- (fx + fy));

    casadi::Function Obstacle("obstacle", {_pos, _origin, _sigma, _amplitude}, {f}, {"pos", "origin", "sigma", "amplitude"}, {"obstacle"});

    return Obstacle;

}

