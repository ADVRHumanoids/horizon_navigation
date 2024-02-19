#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <obstacle.h>

namespace py = pybind11;


template <typename T, typename... Args>
auto make_deserialized(casadi::Function (T::* mem_fn)(Args... args))
{

    // make lambda to create deserialized function
    auto deserialized = [mem_fn](T& self, Args... args)
    {
        // call member function
        auto fn = (self.*mem_fn)(args...);

#if PYBIND11_VERSION_MINOR > 6
        auto cs = py::module_::import("casadi");
#else
        auto cs = py::module::import("casadi");
#endif
        auto Function = cs.attr("Function");
        auto deserialize = Function.attr("deserialize");
        return deserialize(fn.serialize());
    };

    return deserialized;
}

PYBIND11_MODULE(pyObstacle, m) {

    py::class_<Obstacle, Obstacle::Ptr>(m, "Obstacle")
            .def(py::init<>())
            .def("gaussObstacle",
                  make_deserialized(&Obstacle::gaussObstacle))
            .def("simpleObstacle",
                  make_deserialized(&Obstacle::simpleObstacle))

            ;
}
