#include "widget.h"
#include "ui_widget.h"
#include <QDebug>
#include <QSerialPort>
#include <QSerialPortInfo>

#include <QApplication>
#include <QMainWindow>
#include <QTextEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QFile>
#include <QTextStream>
#include <QStringList>
#include <QFileDialog>
#include <QMessageBox>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    qDebug()<<"Inicio el programa: ";

    foreach(const QSerialPortInfo info, QSerialPortInfo::availablePorts()){
        QString pname = info.portName();
        qDebug()<<pname;
        ui->comboBox->addItem(pname);
    }
    ttl = new QSerialPort(this);

    setupPlot();
    ui->progressBar->setRange(0, 200);
    ui->progressBar_2->setRange(0, 200);
    ui->progressBar_3->setRange(0, 200);
    ui->progressBar_4->setRange(0, 200);


}

Widget::~Widget()
{
    delete ui;
}

void Widget::setupPlot()
{
    ui->customPlot->addGraph();
    ui->customPlot->graph(0)->setPen(QPen(Qt::blue));
    ui->customPlot->graph(0)->setName("RPM");

    ui->customPlot->addGraph();
    ui->customPlot->graph(1)->setPen(QPen(Qt::red));
    ui->customPlot->graph(1)->setName("mm/s");


    ui->customPlot->xAxis->setLabel("Tiempo (s)");
    ui->customPlot->xAxis->setRange(0, 10);

    ui->customPlot->yAxis->setLabel("RPM");
    ui->customPlot->yAxis->setRange(0, 400);
    ui->customPlot->yAxis->setLabelColor(Qt::blue);
    ui->customPlot->yAxis->setTickLabelColor(Qt::blue);

    ui->customPlot->yAxis2->setVisible(true);
    ui->customPlot->yAxis2->setLabel("Velocidad (mm/s)");
    ui->customPlot->yAxis2->setRange(0, 800);
    ui->customPlot->yAxis2->setLabelColor(Qt::red);
    ui->customPlot->yAxis2->setTickLabelColor(Qt::red);

    ui->customPlot->graph(1)->setValueAxis(ui->customPlot->yAxis2);

    ui->customPlot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    ui->customPlot->legend->setVisible(true);
    ui->customPlot->legend->setFont(QFont("Helvetica", 9));
    ui->customPlot->legend->setBrush(QBrush(QColor(255, 255, 255, 230)));
    ui->customPlot->axisRect()->insetLayout()->setInsetAlignment(0, Qt::AlignTop | Qt::AlignRight);
}

void Widget::updatePlot()
{
    ui->customPlot->graph(0)->setData(x, y);      // RPM
    ui->customPlot->graph(1)->setData(x, y_mm_s); // mm/s

    ui->customPlot->xAxis->rescale();
    ui->customPlot->xAxis->scaleRange(1.1, ui->customPlot->xAxis->range().center());

    ui->customPlot->yAxis->setRange(0, 400);
    ui->customPlot->yAxis2->setRange(0, 800);
    ui->customPlot->replot();
}

void Widget::readSerial()
{
    QByteArray buffer = ttl->readAll();

    uint8_t a = buffer.at(buffer.length()-1);

    buffer = buffer.left(68);
    qDebug() << "Read: ";
    qDebug() << buffer;
    if (buffer.at(0) == 0x09 && a == 0x07 && buffer.at(1) == 0x43) {

        int idx = 2;

        int16_t RPM = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->valor->setText(QString::number(RPM));
        idx += 3;

        int32_t contador = (static_cast<uint8_t>(buffer.at(idx + 1)) << 24) |
                           (static_cast<uint8_t>(buffer.at(idx + 2)) << 16) |
                           (static_cast<uint8_t>(buffer.at(idx + 3)) << 8) |
                           static_cast<uint8_t>(buffer.at(idx + 4));
        ui->vueltas->setText(QString::number(contador));
        idx += 5;

        uint32_t tiempo = (static_cast<uint8_t>(buffer.at(idx + 1)) << 24) |
                          (static_cast<uint8_t>(buffer.at(idx + 2)) << 16) |
                          (static_cast<uint8_t>(buffer.at(idx + 3)) << 8) |
                          static_cast<uint8_t>(buffer.at(idx + 4));
        ui->tmp->setText(QString::number(tiempo));
        idx += 5;

        int16_t mm_s = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->mm->setText(QString::number(mm_s));
        idx += 3;

        int16_t rad_s = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->rad->setText(QString::number(rad_s));
        idx += 3;

        uint16_t adc1 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->ADC1->setText(QString::number(adc1));
        idx += 3;

        uint16_t adc2 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->ADC2->setText(QString::number(adc2));
        idx += 3;

        uint16_t adc3 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->ADC3->setText(QString::number(adc3));
        idx += 3;

        uint16_t adc4 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->ADC4->setText(QString::number(adc4));
        idx += 3;

        float volt1;
        memcpy(&volt1, buffer.constData() + idx + 1, sizeof(float));
        ui->V1->setText(QString::number(volt1, 'f', 2)); // 2 decimales
        idx += 5;

        float volt2;
        memcpy(&volt2, buffer.constData() + idx + 1, sizeof(float));
        ui->V2->setText(QString::number(volt2, 'f', 2));
        idx += 5;

        float volt3;
        memcpy(&volt3, buffer.constData() + idx + 1, sizeof(float));
        ui->V3->setText(QString::number(volt3, 'f', 2));
        idx += 5;

        float volt4;
        memcpy(&volt4, buffer.constData() + idx + 1, sizeof(float));
        ui->V4->setText(QString::number(volt4, 'f', 2));
        idx += 5;

        uint16_t dist1 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->MM1->setText(QString::number(dist1));
        ui->progressBar->setValue(dist1);
        idx += 3;

        uint16_t dist2 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->MM2->setText(QString::number(dist2));
        ui->progressBar_2->setValue(dist2);
        idx += 3;

        uint16_t dist3 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->MM3->setText(QString::number(dist3));
        ui->progressBar_3->setValue(dist3);
        idx += 3;

        uint16_t dist4 = (static_cast<uint8_t>(buffer.at(idx + 1)) << 8) | static_cast<uint8_t>(buffer.at(idx + 2));
        ui->MM4->setText(QString::number(dist4));
        ui->progressBar_4->setValue(dist4);
        idx += 3;

        double timeInSeconds = tiempo / 1000.0;
        x.append(timeInSeconds);
        y.append(RPM);
        y_mm_s.append(mm_s);

        updatePlot();

        QString logEntry = QString("%1\t%2\n").arg(tiempo).arg(RPM);
        ui->textEdit->append(logEntry);

        if (x.size() > 1000) {
            x.remove(0, x.size() - 1000);
            y.remove(0, y.size() - 1000);
            y_mm_s.remove(0, y_mm_s.size() - 1000);
        }
    }
}


void Widget::on_encendido_clicked()
{
    QByteArray datos;
    datos.append(0x0B);
    ttl->write(datos);
    ui->comando->setText("Motor\nEncendido");

}


void Widget::on_apagado_clicked()
{
    QByteArray datos;
    datos.append(0x0C);
    ttl->write(datos);
    ui->comando->setText("Motor\nApagado");
}


void Widget::on_reversa_clicked()
{
    QByteArray datos;
    datos.append(0x0A);
    ttl->write(datos);
    ui->comando->setText("Reversa");
}



void Widget::on_pushButton_clicked()
{
    if (ttl->isOpen()) {
        ttl->close();
    }

    QString ttl_port_name = ui->comboBox->currentText();

    if (ui->pushButton->text() == "Open") {
        ttl->setPortName(ttl_port_name);
        ttl->open(QSerialPort::ReadWrite);
        ttl->setBaudRate(QSerialPort::Baud115200);
        ttl->setDataBits(QSerialPort::Data8);
        ttl->setFlowControl(QSerialPort::NoFlowControl);
        ttl->setParity(QSerialPort::NoParity);
        ttl->setStopBits(QSerialPort::OneStop);
        QObject::connect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
        ui->pushButton->setText("Close");
    } else {
        ttl->close();
        QObject::disconnect(ttl, SIGNAL(readyRead()), this, SLOT(readSerial()));
        ui->pushButton->setText("Open");
    }
}


void Widget::on_pushButton_2_clicked()
{
    ui->textEdit->clear();
    x.clear();
    y.clear();
    y_mm_s.clear();
    updatePlot();
}

void Widget::on_widget_customContextMenuRequested(const QPoint &pos)
{
    Q_UNUSED(pos);
}




void Widget::on_pushButton_3_clicked()

{

    QString data = ui->textEdit->toPlainText();
    if (data.isEmpty()) {
        QMessageBox::warning(this, "Advertencia", "No hay datos para exportar.");
        return;
    }


    QString filePath = QFileDialog::getSaveFileName(this,"Exportar Datos", QDir::homePath() + "/rpm_data.csv","CSV Files (*.csv)");

    if (filePath.isEmpty()) {
        qDebug() << "Export canceled by user";
        return;
    }

    QFile file(filePath);
    if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QTextStream out(&file);


        out << "\"Time (ms)\";\"RPM\"\n";


        QStringList lines = data.split("\n", QString::SkipEmptyParts);

        for (const QString& line : lines) {
            QStringList values = line.split("\t");
            if (values.size() == 2) {

                out << "\"" << values[0] << "\";\"" << values[1] << "\"\n";
            }
        }

        file.close();
        qDebug() << "Data exported to" << filePath;
        QMessageBox::information(this, "Éxito", "Datos exportados correctamente a:\n" + filePath);
    } else {
        qDebug() << "Failed to export data to" << filePath;
        QMessageBox::warning(this, "Error", "No se pudo exportar los datos a:\n" + filePath);
    }
}


void Widget::on_Enviar_clicked()
{
    if (ttl->isOpen()) {
        bool ok;
        float num = ui->lineEdit->text().toFloat(&ok);
        if (ok) {
            QByteArray data;
            data.append(static_cast<char>(0x07));
            data.append(reinterpret_cast<const char*>(&num), sizeof(float));
            ttl->write(data);
        } else {
            qDebug() << "Error: Entrada no válida";
        }
    } else {
        qDebug() << "Error: El puerto serie no está abierto";
    }
}

void Widget::on_Enviar_2_clicked()
{
    if (ttl->isOpen()) {
        bool ok;
        int32_t temp = ui->lineEdit->text().toInt(&ok);
        if (ok && temp >= -750 && temp <= 750) {
            int16_t num2 = static_cast<int16_t>(temp);
            QByteArray data;
            data.append(static_cast<char>(0x08));
            data.append(static_cast<unsigned char>(num2 >> 8));
            data.append(static_cast<unsigned char>(num2 & 0xFF));
            ttl->write(data);
            qDebug() << "Datos enviados (int16_t): 0x08 + valor =" << num2;
        } else {
            qDebug() << "Error: Entrada no válida o fuera de rango (-750 a 750)";
        }
    } else {
        qDebug() << "Error: El puerto serie no está abierto";
    }
}

void Widget::on_Enviar_3_clicked()
{
    if (ttl->isOpen()) {
        bool ok3;
        int32_t num3 = ui->lineEdit->text().toUInt(&ok3);
        if (ok3) {
            QByteArray data;
            data.append(static_cast<char>(0x09));
            data.append(reinterpret_cast<const char*>(&num3), sizeof(num3));
            ttl->write(data);

        } else {
            qDebug() << "Error: Entrada no válida";
        }
    } else {
        qDebug() << "Error: El puerto serie no está abierto";
    }
}



