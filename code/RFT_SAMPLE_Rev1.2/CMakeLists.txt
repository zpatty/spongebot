cmake_minimum_required(VERSION 3.4...3.18)
project(rft)

find_package(pybind11 REQUIRED)
pybind11_add_module(rft SHARED rft.cpp RFT_IF_PACKET_Rev1.2.cpp RFT_UART_SAMPLE.cpp)