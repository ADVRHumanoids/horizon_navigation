#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <obstacles_generator.h>

namespace py = pybind11;


PYBIND11_MODULE(pyObstacleGenerator, m) {

    py::class_<ObstacleGenerator, ObstacleGenerator::Ptr>(m, "ObstacleGenerator")
            .def(py::init<double, double, double, std::vector<std::string>>())
            .def("run", &ObstacleGenerator::run)
            .def("setMaxObstacleNum", &ObstacleGenerator::setMaxObstacleNum)
            .def("setObstacleRadius", &ObstacleGenerator::setObstacleRadius)
            .def("getObstacles", &ObstacleGenerator::getObstacles)
            .def("setBlindAngle", &ObstacleGenerator::setBlindAngle)
            .def("setAngleThreshold", &ObstacleGenerator::setAngleThreshold)
            ;
}
