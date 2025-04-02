#ifndef WIDGET_H
#define WIDGET_H
#include <QWidget>
#include <QDebug>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QVector>
#include "qcustomplot.h"



QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE




class Widget : public QWidget
{
    Q_OBJECT

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();






private slots:

    void on_encendido_clicked();

    void on_apagado_clicked();

    void readSerial();

    void on_Enviar_clicked();

    void on_reversa_clicked();

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

    void on_widget_customContextMenuRequested(const QPoint &pos);

    void on_pushButton_3_clicked();

    void on_Enviar_2_clicked();

    void on_Enviar_3_clicked();

private:
    Ui::Widget *ui;
    int enviarrpm;
    uint RPM;
    uint RPMant=0;
    uint TIEMPO=0;
    uint TIEMPOMAX=0;
    int contador=0;
    int bandera1=0;
    QSerialPort*ttl=nullptr;
    void setupPlot();
    //double x[100],y[100];
    void updatePlot();
    QVector<double> x, y; // Para tiempo y RPM
    QVector<double> y_mm_s; // Nuevo vector para mm/s
    QVector<double> z;
    QVector<double> w;


};
#endif // WIDGET_H

