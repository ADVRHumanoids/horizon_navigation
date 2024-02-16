#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <math.h>

//namespace casadi {
//class Function;
//}

class Obstacle
{
    public:

        Obstacle();

//        double amplitude,
//        Eigen::Ref<const Eigen::Vector2d> sigma

        casadi::Function obstacleFunction();

    private:

//        typedef casadi::SX Scalar;
//        typedef Eigen::Matrix<Scalar, -1, 1>  VectorXs;

//        static VectorXs cas_to_eig(const casadi::SX& cas);
//        static casadi::SX eig_to_cas(const VectorXs& eig);

        casadi::SX _pos;
        casadi::SX _origin;
        casadi::SX _sigma;
        casadi::SX _amplitude;
//        Eigen::Ref<Eigen::Vector2d> _sigma;

};

#endif // OBSTACLE_H
