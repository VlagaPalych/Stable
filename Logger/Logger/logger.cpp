#include "logger.h"
#include "ui_logger.h"
#include <QDebug>
#include <QString>
#include <QList>
#include <QSerialPort>
#include <QSerialPortInfo>
#include "serialportreader.h"
#include <QFileDialog>


Logger::Logger(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::Logger)
{
    ui->setupUi(this);

    QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
    foreach (QSerialPortInfo port, availablePorts) {
        ui->serialComboBox->addItem(port.portName());
    }
    ui->serialComboBox->setCurrentText("COM12");

    connect(ui->controlButton, SIGNAL(clicked()), SLOT(controlButtonToggled()));
    connect(ui->stopButton, SIGNAL(clicked()), SLOT(handleStopButton()));
    connect(ui->spinBox, SIGNAL(valueChanged(int)), SLOT(handleSpinBox(int)));
    connect(ui->reloadButton, SIGNAL(clicked()), SLOT(handleReloadButton()));
    connect(ui->connectButton, SIGNAL(clicked()), SLOT(handleConnectButton()));

    stmReader = NULL;
}

void Logger::defineLogFile() {
    QDir curDir = QDir::current();
    int maxLogNumber = 0;
    foreach (QString entry, curDir.entryList()) {
        if (entry.startsWith(tr("log"))) {
            int txtIndex = entry.indexOf(".txt");
            int logNumber = entry.mid(3, txtIndex - 3).toInt();
            if (logNumber > maxLogNumber) {
                maxLogNumber = logNumber;
            }
        }
    }
    maxLogNumber++;
    stmReader = new SerialPortReader(stm, tr("log") + QString::number(maxLogNumber) + tr(".txt"));
}

void Logger::handleConnectButton() {
    stm = new QSerialPort(ui->serialComboBox->currentText());
    stm->setBaudRate(QSerialPort::Baud9600);
    stm->setParity(QSerialPort::NoParity);
    stm->setStopBits(QSerialPort::OneStop);
    if (!stm->open(QIODevice::ReadWrite)) {
        qDebug() << QObject::tr("Failed to open port %1, error: %2").arg(stm->portName()).arg(stm->errorString()) << endl;
    }

    defineLogFile();

    ui->stopButton->setEnabled(true);
    ui->controlButton->setEnabled(true);
    ui->reloadButton->setEnabled(true);
    ui->spinBox->setEnabled(true);
}

Logger::~Logger()
{
    delete ui;
    if (stmReader) {
        delete stmReader;
    }
    delete stm;
}

void Logger::controlButtonToggled() {
    //stm->write("p2000e");
    stm->write("b");
}

void Logger::handleStopButton() {
    //stm->write("p1000e");
    stm->write("s");
}

void Logger::handleSpinBox(int value) {
    stm->write(tr("p%1e").arg(value).toStdString().c_str());
}


void Logger::handleReloadButton() {
    stm->write("r");
}
