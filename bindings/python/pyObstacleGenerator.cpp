#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <obstacles_generator.h>

namespace py = pybind11;


PYBIND11_MODULE(pyobstacleGenerator, m) {

    py::class_<ObstacleGenerator, ObstacleGenerator::Ptr>(m, "ObstacleGenerator")
            .def(py::init<>())
            .def("gaussObstacle", &ObstacleGenerator::gaussObstacle)
//            .def("addPhase", static_cast<bool (SinglePhaseManager::*)(std::vector<Phase::Ptr>, int, bool)>(&SinglePhaseManager::addPhase), py::arg("phases"), py::arg("pos") = -1, py::arg("absolute_position") = false)
//            .def("addPhase", static_cast<bool (SinglePhaseManager::*)(Phase::Ptr, int, bool)>(&SinglePhaseManager::addPhase), py::arg("phase"), py::arg("pos") = -1, py::arg("absolute_position") = false)
//            .def("getRegisteredPhase", &SinglePhaseManager::getRegisteredPhase)
//            .def("getRegisteredPhases", &SinglePhaseManager::getRegisteredPhases)
//            .def("getEmptyNodes", &SinglePhaseManager::getEmptyNodes)
//            .def("getActivePhases", &SinglePhaseManager::getActivePhases)
//            .def("getPhases", &SinglePhaseManager::getPhases)
//            .def("shift", &SinglePhaseManager::shift)
//            .def("clear", &SinglePhaseManager::clear)
            ;
}
