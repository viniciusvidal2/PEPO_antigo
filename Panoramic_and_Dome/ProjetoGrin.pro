#-------------------------------------------------
#
# Project created by QtCreator 2019-10-03T10:26:36
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ProjetoGrin
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
INCLUDEPATH += C:"/home/grin/opencv-3.1.0/releaseinstall/include"

LIBS += -L/"C:/home/grin/opencv-3.1.0/release/lib"
LIBS += -L/"C:/home/grin/opencv-3.1.0/release/lib/libopencv_highgui300.dll.a"
LIBS += -L/"C:/home/grin/opencv-3.1.0/release/lib/libopencv_imgproc300.dll.a"
LIBS += -L/"C:/home/grin/opencv-3.1.0/release/lib/libopencv_features2d300.dll.a"
LIBS += -L/"C:/home/grin/opencv-3.1.0/release/lib/libopencv_calib3d300.dll.a"
LIBS += -lopencv_core
LIBS += -lopencv_imgproc
LIBS += -lopencv_highgui
LIBS +=  -lopencv_stitching
LIBS += -lopencv_calib3d
LIBS += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui -lopencv_xfeatures2d -lopencv_features2d

LIBS +=  -lopencv_stitching

SOURCES += \
        main.cpp \
    importar.cpp

HEADERS += \
    main.hpp

FORMS +=

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
