#include "boardconsole.h"
#include <QtSerialPort\qserialportinfo.h>
#include <qdir.h>

BoardConsole::BoardConsole(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
	foreach(QSerialPortInfo port, availablePorts) {
		ui.serialComboBox->addItem(port.portName());
	}
	//ui.serialComboBox->setCurrentText("COM12");

	connect(ui.connectButton, SIGNAL(clicked()), SLOT(handleConnectButton()));
	connect(ui.stabOnButton, SIGNAL(clicked()), SLOT(handleStabOnButton()));
	connect(ui.stabOffButton, SIGNAL(clicked()), SLOT(handleStabOffButton()));
	connect(ui.calibrButton, SIGNAL(clicked()), SLOT(handleCalibrButton()));
	connect(ui.fullRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.accelRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.telemetryStartButton, SIGNAL(clicked()), SLOT(handleTelemetryStartButton()));
	connect(ui.telemetryStopButton, SIGNAL(clicked()), SLOT(handleTelemetryStopButton()));

	stm = NULL;
	stmReader = NULL;

	acceleration = QVector<QwtPlotCurve *>(6);
	accelData = QVector<QVector<double> >(6);
	for (int i = 0; i < acceleration.size(); i++) {
		acceleration[i] = new QwtPlotCurve;
		accelData[i] = QVector<double>();
	}
	xData = QVector<double>();
}

BoardConsole::~BoardConsole() {
	if (stmReader)	delete stmReader;
	if (stm)		delete stm;

	for (int i = 0; i < 6; i++) {
		delete acceleration[i];
	}
}

void BoardConsole::defineLogFile() {
	QDir curDir = QDir::current();
	int maxLogNumber = 0;
	foreach(QString entry, curDir.entryList()) {
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
	connect(stmReader, SIGNAL(freshLine(QString &)), SLOT(handleFreshLine(QString &)));
}

void BoardConsole::handleConnectButton() {
	stm = new QSerialPort(ui.serialComboBox->currentText());
	stm->setBaudRate(QSerialPort::Baud9600);
	stm->setParity(QSerialPort::NoParity);
	stm->setStopBits(QSerialPort::OneStop);
	if (!stm->open(QIODevice::ReadWrite)) {
		qDebug() << QObject::tr("Failed to open port %1, error: %2").arg(stm->portName()).arg(stm->errorString()) << endl;
	}

	defineLogFile();
}

void BoardConsole::handleStabOnButton() {
	stm->write("e");
}

void BoardConsole::handleStabOffButton() {
	stm->write("f");
}

void BoardConsole::handleCalibrButton() {
	stm->write("g");
}

void BoardConsole::handleTelemetryButtons() {
	if (ui.fullRadioButton->isChecked()) {
		stm->write("k");
	} else if (ui.accelRadioButton->isChecked()) {
		stm->write("l");
	}
}	

void BoardConsole::handleTelemetryStartButton() {
	stm->write("h");
}

void BoardConsole::handleTelemetryStopButton() {
	stm->write("i");
}

void BoardConsole::handleFreshLine(QString &line) {
	if (ui.fullRadioButton->isChecked()) {
	}
	else if (ui.accelRadioButton->isChecked()) {
		QStringList numbers = line.split(' ');
		
		for (int i = 0; i < accelData.size(); i++) {
			xData.append(xData.size() + 1);
			accelData[i].append(numbers[i].toDouble());
			acceleration[i]->setSamples(xData, accelData[i]);
		}

	}
}