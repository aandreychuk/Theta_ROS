TARGET = Theta_ROS
CONFIG   += console
CONFIG   -= app_bundle
TEMPLATE = app
QMAKE_CXXFLAGS += -std=c++11

win32 {
QMAKE_LFLAGS += -static -static-libgcc -static-libstdc++
}

SOURCES += \
    map.cpp \
    theta.cpp \
    main.cpp

HEADERS += \
    map.h \
    theta.h \
    structs.h
