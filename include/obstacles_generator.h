#ifndef OBSTACLESGENERATOR_H
#define OBSTACLESGENERATOR_H

// #include <Eigen/Dense>
// #include <numeric>
// #include <vector>
// #include <iostream>
#include <memory>
#include <casadi/casadi.hpp>
// #include <stdexcept>
// #include <unordered_set>
//#include <any>
//#include <variant>
//#include <functional>

class ObstacleGenerator
{

public:

    ObstacleGenerator();
    typedef std::shared_ptr<ObstacleGenerator> Ptr;

    casadi::Function gaussObstacle();
    casadi::Function simpleObstacle();

private:




};

#endif
