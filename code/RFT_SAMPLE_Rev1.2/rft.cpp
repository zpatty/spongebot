// pywrap.cpp
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "RFT_UART_SAMPLE.h"
#include "RFT_IF_PACKET_Rev1.2.h"

namespace py = pybind11;
constexpr auto byref = py::return_value_policy::reference_internal;

PYBIND11_MODULE(rft, m) {
    m.doc() = "optional module docstring";

    py::class_<CRT_RFT_UART> rft_uart(m, "CRT_RFT_UART");

        rft_uart.def(py::init<>())  
        .def("openPort", &CRT_RFT_UART::openPort)
        .def("rqst_FT_Continuous", &CRT_RFT_UART::rqst_FT_Continuous)
        .def("rqst_FT_Stop", &CRT_RFT_UART::rqst_FT_Stop)
        .def("set_FT_Bias", &CRT_RFT_UART::set_FT_Bias)
        .def("set_Comm_Speed", &CRT_RFT_UART::set_Comm_Speed)
        .def("closePort", &CRT_RFT_UART::closePort)
        .def("set_FT_Cont_Interval", &CRT_RFT_UART::set_FT_Cont_Interval)
        .def("rcvd_FT", &CRT_RFT_UART::rcvd_FT)
        .def_readonly("force_out", &CRT_RFT_UART::force_out);
        // .def_readwrite("m_RFT_IF_PACKET", &CRT_RFT_UART::m_RFT_IF_PACKET);

    // py::class_<CRT_RFT_IF_PACKET>(m, "CRT_RFT_IF_PACKET")
    //     .def(py::init<>())
    //     .def("getForce", &CRT_RFT_IF_PACKET::getForce);
        // .def_readonly("m_rcvdForce", &CRT_RFT_IF_PACKET::m_rcvdForce);

    // py::class_<CRT_RFT_IF_PACKET>(m, "CRT_RFT_IF_PACKET", py::dynamic_attr())
    // .def(py::init<>())
    // .def("setDivider", &CRT_RFT_IF_PACKET::setDivider)
    // .def_readonly("m_rcvdForce", &CRT_RFT_IF_PACKET::m_rcvdForce)
    // .def_readonly("m_rcvd_tx_frq", &CRT_RFT_IF_PACKET::m_rcvd_tx_frq);
}