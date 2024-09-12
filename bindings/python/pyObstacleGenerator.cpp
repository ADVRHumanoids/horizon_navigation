#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <obstacles_generator.h>

namespace py = pybind11;


PYBIND11_MODULE(pyObstacleGenerator, m) {

    py::class_<ObstacleGenerator, ObstacleGenerator::Ptr>(m, "ObstacleGenerator")
            .def(py::init<double, double, double, std::string, std::string>(), py::arg("grid_height"), 
                                                                                            py::arg("grid_width"),
                                                                                            py::arg("grid_resolution"),
                                                                                            py::arg("input_topic_name") = "/map",
                                                                                            py::arg("rviz_markers_topic_name") = ""
                                                                                            )
            .def("run", &ObstacleGenerator::run)
            .def("setMaxObstacleNum", &ObstacleGenerator::setMaxObstacleNum)
            .def("setObstacleRadius", &ObstacleGenerator::setObstacleRadius)
            .def("getObstacles", &ObstacleGenerator::getObstacles)
            .def("setBlindAngle", &ObstacleGenerator::setBlindAngle)
            .def("setAngleThreshold", &ObstacleGenerator::setAngleThreshold)
            ;
}
