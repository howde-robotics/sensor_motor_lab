#-------------------------------------------------
#
# Project created by QtCreator 2021-02-09T13:31:41
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets printsupport

INCLUDEPATH += /opt/ros/${ROS_DISTRO}/include

LIBS += -L/opt/ros/${ROS_DISTRO}/lib -lroscpp -lroslib -lrosconsole -lroscpp_serialization -lrostime -lrviz

TARGET = sensor_motor_lab_gui
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

CONFIG += c++11

SOURCES += \
        main.cpp \
        mainwindow.cpp \
        publishernode.cpp \
        qcustomplot.cpp \
        subscribernode.cpp

HEADERS += \
        mainwindow.h \
        msgs.h \
        publishernode.h \
        qcustomplot.h \
        subscribernode.h

FORMS += \
        mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
