QT -= gui

CONFIG += c++14 console
CONFIG -= app_bundle

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS #DEBUG_FIBRE

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0
INCLUDEPATH += \
        fibre/cpp/include

SOURCES += \
        fibre/cpp/posix_tcp.cpp \
        fibre/cpp/protocol.cpp \
        main.cpp \
    my_thread.cpp \
    fibre/cpp/posix_udp.cpp \
    MotorControl/axis.cpp \
    MotorControl/controller.cpp \
    MotorControl/encoder.cpp \
    MotorControl/motor.cpp \
    MotorControl/sensorless_estimator.cpp \
    MotorControl/trapTraj.cpp \
    MotorControl/utils.c

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

HEADERS += \
    my_thread.h \
    fibre/cpp/include/fibre/posix_udp.hpp \
    fibre/cpp/include/fibre/cpp_utils.hpp \
    fibre/cpp/include/fibre/crc.hpp \
    fibre/cpp/include/fibre/decoders.hpp \
    fibre/cpp/include/fibre/encoders.hpp \
    fibre/cpp/include/fibre/posix_tcp.hpp \
    fibre/cpp/include/fibre/protocol.hpp \
    MotorControl/axis.hpp \
    MotorControl/controller.hpp \
    MotorControl/encoder.hpp \
    MotorControl/motor.hpp \
    MotorControl/sensorless_estimator.hpp \
    MotorControl/trapTraj.hpp \
    MotorControl/utils.h \
    MotorControl/odrive_main.h


LIBS += -lws2_32
