# Projeto ESP-IDF 4.4 - Controladora de voo do Drone (porte em C puro).
# Build: . $IDF_PATH/export.sh && idf.py set-target esp32 && idf.py build
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_fc)
