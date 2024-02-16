#include "obstacle.h"

Obstacle::Obstacle(Eigen::Vector2d origin)

//double amplitude,
//Eigen::Ref<const Eigen::Vector2d> sigma
{
    casadi_int dim = 2;
    _pos = casadi::SX::sym("pos", dim);
    _origin = casadi::SX::sym("origin", dim);
    _sigma = casadi::SX::sym("sigma", dim);
    _amplitude = casadi::SX::sym("amplitude", 1);

}

//Obstacle::VectorXs Obstacle::cas_to_eig(const casadi::SX & cas)
//{
//    VectorXs eig(cas.size1());
//    for(int i = 0; i < eig.size(); i++)
//    {
//        eig(i) = cas(i);
//    }
//    return eig;
//}

//casadi::SX Obstacle::eig_to_cas(const Obstacle::VectorXs & eig)
//{
//    auto sx = casadi::SX(casadi::Sparsity::dense(eig.size()));
//    for(int i = 0; i < eig.size(); i++)
//    {
//        sx(i) = eig(i);
//    }
//    return sx;

//}

casadi::Function Obstacle::createObstacle()
{
    casadi::SX cazz = casadi::SX::sym("cazz");
//    casadi::SX cazzi = casadi::pow(cazz, 2) + 10;
    auto cazzi = casadi::pow(cazz, 2);
    // simple gaussian curve
//    auto fx = _pos(0) - _origin(0);
//    auto cazzi = casadi::pow(_pos(0) - _origin(0), 2);
//    auto fx = casadi::pow(_pos(0) - _origin(0), 2) / (2 * std::pow(_sigma(0), 2));
//    auto fy = casadi::pow(_pos(1) - _origin(1), 2) / (2 * std::pow(_sigma(1), 2));

//    auto f = _amplitude * (- (fx + fy));

//    casadi::Function Obstacle("obstacle", {_pos, _origin, _sigma, _amplitude}, {f}, {"pos", "origin", "sigma", "amplitude"}, {"obstacle"});

//    return Obstacle;

}


//    # x-pos for gap crossing
//    orig = 1.
//    lims = 0.1  # 0.25   #0.1#
//    slope = 20  # 5  # 10

//    exp1 = 1/np.exp(-slope*(p[0]-orig-lims))
//    exp2 = 1/np.exp(slope*(p[0]-orig+lims))
//    f_mountain = 1 / (exp1 + exp2)

//    + 1/ (e^(4∙(x −4 − 0.5))
