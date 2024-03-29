TEMPLATE = app
TARGET = demo
DEPENDPATH += . release
INCLUDEPATH += . ../../libpedsim
DESTDIR = ../

QMAKE_CXXFLAGS += -std=c++0x -Wno-deprecated-register

# Remove -lcudart if your system does not support CUDA
LIBS += -L../../libpedsim -lpedsim
LIBS += -L/opt/AMDAPPSDK-3.0-0-Beta/libs/x86_64 
LIBS += -lOpenCL
QT += opengl
QT += widgets

CONFIG += release
CONFIG += console

# Input
HEADERS += MainWindow.h ParseScenario.h  ViewAgent.h Timer.h
SOURCES += main.cpp MainWindow.cpp ParseScenario.cpp ViewAgent.cpp Timer.cpp
