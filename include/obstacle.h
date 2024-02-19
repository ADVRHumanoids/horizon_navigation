#ifndef OBSTACLE_H
#define OBSTACLE_H

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

class Obstacle
{

public:

    Obstacle();
    typedef std::shared_ptr<Obstacle> Ptr;

    casadi::Function gaussObstacle();
    casadi::Function simpleObstacle();

private:

};

#endif
