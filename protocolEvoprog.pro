QT += core
QT -= gui

CONFIG += c++11

TARGET = protocolEvoprog
CONFIG += console
CONFIG -= app_bundle

TEMPLATE = app

SOURCES += main.cpp

INCLUDEPATH += X:/boost_1_61_0
INCLUDEPATH += X:/EvoCoreLibrary/include
INCLUDEPATH += X:/EvoCoreLibrary/lib
INCLUDEPATH += C:/Python27/include

LIBS += -L$$quote(X:/boost_1_61_0/stage/lib) -lboost_python-vc140-mt-1_61
LIBS += -L$$quote(C:/Python27/libs)
LIBS += -L$$quote(X:/EvoCoreLibrary/lib) -lEvoCoderCore

QMAKE_CXXFLAGS += -bigobj
