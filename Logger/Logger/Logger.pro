#-------------------------------------------------
#
# Project created by QtCreator 2015-04-28T19:00:16
#
#-------------------------------------------------

QT       += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Logger
TEMPLATE = app


SOURCES += main.cpp\
        logger.cpp \
    serialportreader.cpp

HEADERS  += logger.h \
    serialportreader.h

FORMS    += logger.ui
