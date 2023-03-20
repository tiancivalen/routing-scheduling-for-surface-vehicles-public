#-------------------------------------------------
#
# Project created by QtCreator 2013-09-23T14:36:31
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = p1
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    cregion.cpp \
    cregioncollect.cpp \
    operation.cpp \
    twindow.cpp \
    globalvariable.cpp \
    cnode.cpp \
    vehicle.cpp \
    base_step.cpp \
    ctaxiwaymodel.cpp \
    cnodecollect.cpp \
    cconfig.cpp \
    twindow_r.cpp \
    nowaiting_step.cpp \
    TP_nowaitingExt.cpp \
    TP_robust_globalvariables.cpp \
    utils.cpp

HEADERS  += mainwindow.h \
    cregion.h \
    common.h \
    cregioncollect.h \
    operation.h \
    twindow.h \
    globalvariable.h \
    cnode.h \
    vehicle.h \
    base_step.h \
    ctaxiwaymodel.h \
    cnodecollect.h \
    cconfig.h \
    twindow_r.h \
    nowaiting_step.h \
    TP_nowaiting_include.h \
    TP_nowaitingExt.h \
    TP_robust_globalvariables.h \
    utils.h

FORMS    += mainwindow.ui

RESOURCES +=  icon.qrc
