/******************************************************************************
Copyright (c) 2022, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_python_interface/PybindMacros.h>
#include "ocs2_mpcnet_core/MpcnetInterfaceBase.h"

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(ocs2::size_array_t)

PYBIND11_MAKE_OPAQUE(ocs2::scalar_array_t)

PYBIND11_MAKE_OPAQUE(ocs2::vector_array_t)

PYBIND11_MAKE_OPAQUE(ocs2::matrix_array_t)

PYBIND11_MAKE_OPAQUE(std::vector<ocs2::SystemObservation>)

PYBIND11_MAKE_OPAQUE(std::vector<ocs2::ModeSchedule>)

PYBIND11_MAKE_OPAQUE(std::vector<ocs2::TargetTrajectories>)

PYBIND11_MAKE_OPAQUE(ocs2::mpcnet::data_array_t)

PYBIND11_MAKE_OPAQUE(ocs2::mpcnet::metrics_array_t)

PYBIND11_MODULE(MpcnetPybindings, m) {
    VECTOR_TYPE_BINDING(ocs2::size_array_t, "size_array")
    VECTOR_TYPE_BINDING(ocs2::scalar_array_t, "scalar_array")
    VECTOR_TYPE_BINDING(ocs2::matrix_array_t, "matrix_array")

    VECTOR_TYPE_BINDING(std::vector<ocs2::SystemObservation>, "SystemObservationArray")
    VECTOR_TYPE_BINDING(std::vector<ocs2::ModeSchedule>, "ModeScheduleArray")
    VECTOR_TYPE_BINDING(std::vector<ocs2::TargetTrajectories>, "TargetTrajectoriesArray")

    VECTOR_TYPE_BINDING(ocs2::mpcnet::data_array_t, "DataArray")
    VECTOR_TYPE_BINDING(ocs2::mpcnet::metrics_array_t, "MetricsArray")

    py::class_<ocs2::vector_array_t>(m, "vector_array")
            .def(pybind11::init<>())
            .def("clear", &ocs2::vector_array_t::clear)
            .def("pop_back", &ocs2::vector_array_t::pop_back)
            .def("push_back", [](ocs2::vector_array_t &v, const py::array_t<double> &array) {
                auto info = array.request();
                if (info.ndim != 1) {
                    throw std::runtime_error("Incompatible buffer dimension!");
                }
                v.emplace_back(Eigen::Map<Eigen::VectorXd>(static_cast<double *>(info.ptr), info.shape[0]));
            })
            .def("resize", [](ocs2::vector_array_t &v, size_t i) { v.resize(i); })
            .def("__getitem__",
                 [](const ocs2::vector_array_t &v, size_t i) {
                     if (i >= v.size()) throw pybind11::index_error();
                     return v[i];
                 })
            .def("__setitem__",
                 [](ocs2::vector_array_t &v, size_t i, const py::array_t<double> &array) {
                     if (i >= v.size()) {
                         throw pybind11::index_error();
                     }
                     auto info = array.request();
                     if (info.ndim != 1) {
                         throw std::runtime_error("Incompatible buffer dimension!");
                     }
                     v[i] = Eigen::Map<Eigen::VectorXd>(static_cast<double *>(info.ptr), info.shape[0]);
                 })
            .def("__len__", [](const ocs2::vector_array_t &v) { return v.size(); })
            .def("__iter__", [](ocs2::vector_array_t &v) { return pybind11::make_iterator(v.begin(), v.end()); },
                 pybind11::keep_alive<0, 1>());

    /* bind approximation classes */
    py::class_<ocs2::ScalarFunctionQuadraticApproximation>(m, "ScalarFunctionQuadraticApproximation")
            .def_readwrite("f", &ocs2::ScalarFunctionQuadraticApproximation::f)
            .def_readwrite("dfdx", &ocs2::ScalarFunctionQuadraticApproximation::dfdx)
            .def_readwrite("dfdu", &ocs2::ScalarFunctionQuadraticApproximation::dfdu)
            .def_readwrite("dfdxx", &ocs2::ScalarFunctionQuadraticApproximation::dfdxx)
            .def_readwrite("dfdux", &ocs2::ScalarFunctionQuadraticApproximation::dfdux)
            .def_readwrite("dfduu", &ocs2::ScalarFunctionQuadraticApproximation::dfduu);

    /* bind system observation struct */
    py::class_<ocs2::SystemObservation>(m, "SystemObservation")
            .def(pybind11::init<>())
            .def_readwrite("mode", &ocs2::SystemObservation::mode)
            .def_readwrite("time", &ocs2::SystemObservation::time)
            .def_property("state",
                          [](const ocs2::SystemObservation &s) {
                              return s.state;
                          },
                          [](ocs2::SystemObservation &s, const py::array_t<double> &array) {
                              const py::buffer_info info = array.request();
                              if (info.ndim != 1) {
                                  throw std::runtime_error("Incompatible buffer dimension!");
                              }
                              auto ptr = static_cast<double *>(info.ptr);
                              s.state = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> >(ptr, info.shape[0]);
                          })
            .def_property("input",
                          [](const ocs2::SystemObservation &s) {
                              return s.input;
                          },
                          [](ocs2::SystemObservation &s, const py::array_t<double> &array) {
                              const py::buffer_info info = array.request();
                              if (info.ndim != 1) {
                                  throw std::runtime_error("Incompatible buffer dimension!");
                              }
                              auto ptr = static_cast<double *>(info.ptr);
                              s.input = Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, 1> >(ptr, info.shape[0]);
                          });

    /* bind mode schedule struct */
    py::class_<ocs2::ModeSchedule>(m, "ModeSchedule")
            .def(pybind11::init<ocs2::scalar_array_t, ocs2::size_array_t>())
            .def_readwrite("eventTimes", &ocs2::ModeSchedule::eventTimes)
            .def_readwrite("modeSequence", &ocs2::ModeSchedule::modeSequence);

    /* bind target trajectories class */
    py::class_<ocs2::TargetTrajectories>(m, "TargetTrajectories")
            .def(pybind11::init<ocs2::scalar_array_t, ocs2::vector_array_t, ocs2::vector_array_t>())
            .def_readwrite("timeTrajectory", &ocs2::TargetTrajectories::timeTrajectory)
            .def_readwrite("stateTrajectory", &ocs2::TargetTrajectories::stateTrajectory)
            .def_readwrite("inputTrajectory", &ocs2::TargetTrajectories::inputTrajectory);

    /* bind data point struct */
    py::class_<ocs2::mpcnet::data_point_t>(m, "DataPoint")
            .def(pybind11::init<>())
            .def_readwrite("mode", &ocs2::mpcnet::data_point_t::mode)
            .def_readwrite("t", &ocs2::mpcnet::data_point_t::t)
            .def_readwrite("x", &ocs2::mpcnet::data_point_t::x)
            .def_readwrite("u", &ocs2::mpcnet::data_point_t::u)
            .def_readwrite("observation", &ocs2::mpcnet::data_point_t::observation)
            .def_readwrite("actionTransformation", &ocs2::mpcnet::data_point_t::actionTransformation)
            .def_readwrite("hamiltonian", &ocs2::mpcnet::data_point_t::hamiltonian);

    /* bind metrics struct */
    py::class_<ocs2::mpcnet::metrics_t>(m, "Metrics")
            .def(pybind11::init<>())
            .def_readwrite("survivalTime", &ocs2::mpcnet::metrics_t::survivalTime)
            .def_readwrite("incurredHamiltonian", &ocs2::mpcnet::metrics_t::incurredHamiltonian);
}
