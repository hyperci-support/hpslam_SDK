TEMPLATE	= app
HEADERS		= ../include/controlcenter.h \
    ../include/cyusb.h
FORMS		+= controlcenter.ui
SOURCES		= controlcenter.cpp main.cpp fx2_download.cpp fx3_download.cpp
LIBS		+= -L../lib -lcyusb -lusb-1.0
QT		+= network widgets qml quick xml gui svg
TARGET		= ../bin/cyusb_linux
greaterThan(QT_MAJOR_VERSION, 4):QT += widgets

DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x000000
