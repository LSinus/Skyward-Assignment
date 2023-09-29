cmake_minimum_required(VERSION 3.10)

project(Assignement)
set(CMAKE_C_STANDARD 11)

add_executable(grader flight.c grader.c grader.h include/madgwickFilter.c include/madgwickFilter.h)