#ifndef OBSTACLE_H
#define OBSTACLE_H

 #include <Eigen/Dense>
#include <memory>
#include <casadi/casadi.hpp>
// #include <numeric>
// #include <vector>
// #include <iostream>
// #include <stdexcept>
// #include <unordered_set>
//#include <any>
//#include <variant>
//#include <functional>

class Obstacle {
public:

    typedef std::shared_ptr<Obstacle> Ptr;
    virtual Eigen::Vector3d getOrigin() = 0;
//    virtual double getDistance() = 0;

};


class SphereObstacle {
public:

    typedef std::shared_ptr<Obstacle> Ptr;
    virtual Eigen::Vector3d getOrigin() override;
//    virtual double getDistance() = 0;

};

class CasadiObstacle: public Obstacle
{

public:

    typedef std::shared_ptr<CasadiObstacle> Ptr;
    CasadiObstacle(Eigen::Vector3d origin, Eigen::Vector3d radius);

    casadi::Function gaussFormulation();
    casadi::Function simpleFormulation();

    Eigen::Vector3d getOrigin() override;
//    double getDistance() override;
    Eigen::Vector3d getRadius();

private:

    Eigen::Vector3d _origin;
    Eigen::Vector3d _radius;

//    double _distance;

};

#endif
