#-------------------------------------------------
#
# Project created by QtCreator 2019-02-27T10:08:09
#
#-------------------------------------------------

QT += core gui network

CONFIG += c++17

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = bird
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000
# disables all the APIs deprecated before Qt 6.0.0

SOURCES +=\
    dialog_add_label.cc \
    dialog_add_species.cc \
    dialog_modify_name.cc \
        main_window.cc \
    main.cc

HEADERS  += main_window.h \
    codelibrary/base/log.h \
    dialog_add_label.h \
    dialog_add_species.h \
    dialog_modify_name.h \
    taxonomic_tree.h \
    codelibrary/base/array.h

FORMS    += main_window.ui \
    dialog_add_label.ui \
    dialog_add_species.ui \
    dialog_modify_name.ui

DISTFILES +=

RESOURCES += \
    resource.qrc

RC_ICONS = app.ico
