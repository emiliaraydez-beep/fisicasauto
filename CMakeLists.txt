cmake_minimum_required(VERSION 3.15)
project(VehiculoSim CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(NUCLEO src/Vehiculo.cpp src/VehiculoHover.cpp)

if(WIN32)
  # GTA SA es de 32 bits: configurar con  -A Win32
  add_library(VehiculoSim SHARED ${NUCLEO} src/MundoSA.cpp src/plugin.cpp src/exports.def)
  target_include_directories(VehiculoSim PRIVATE src)
  target_compile_definitions(VehiculoSim PRIVATE NOMINMAX WIN32_LEAN_AND_MEAN)
else()
  # En Linux solo se prueba el nucleo con un suelo plano
  add_executable(sim_test test/sim_test.cpp test/MundoPlano.cpp ${NUCLEO})
  target_include_directories(sim_test PRIVATE src)
endif()
