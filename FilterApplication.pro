#-------------------------------------------------
#
# Project created by QtCreator 2016-02-25T09:39:55
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = FilterApplication
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui

INCLUDEPATH += /usr/local/include/opencv2

LIBS    += /usr/local/lib/libopencv_*.so.2.4.11
#LIBS    += /usr/local/lib/libopencv_core.so.2.4.11
#        += /usr/local/lib/libopencv_imgproc.so.2.4.11
#        += /usr/local/lib/libopencv_photo.so.2.4.11



