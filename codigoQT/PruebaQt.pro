QT += core gui
QT += printsupport

QT += core gui serialport

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG   += c++11



# You can make your code fail to compile if it uses deprecated APIs.
# Uncomment the following line to disable deprecated APIs before Qt 6.0.0.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000

SOURCES += \
    #../../../Downloads/PlotDemo/PlotDemo/qcustomplot.cpp \
    main.cpp \
    qcustomplot.cpp\
    widget.cpp

HEADERS += \
    #../../../Downloads/PlotDemo/PlotDemo/qcustomplot.h \
    widget.h \
    qcustomplot.h

FORMS += \
    widget.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target
