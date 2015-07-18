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
	connect(ui.stabToggleButton, SIGNAL(clicked()), SLOT(handleStabToggleButton()));
	connect(ui.calibrButton, SIGNAL(clicked()), SLOT(handleCalibrButton()));
	connect(ui.fullRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.axRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.ayRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.azRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.telemetryToggleButton, SIGNAL(clicked()), SLOT(handleTelemetryToggleButton()));
	connect(ui.HZ100RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));
	connect(ui.HZ800RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));

	connect(ui.noFilterCheckBox, SIGNAL(clicked()), SLOT(handleNoFilterCheckBox()));
	connect(ui.kalmanFilterCheckBox, SIGNAL(clicked()), SLOT(handleKalmanFilterCheckBox()));
	connect(ui.averagingCheckBox, SIGNAL(clicked()), SLOT(handleAveragingCheckBox()));

	stm = NULL;
	stmReader = NULL;

	acceleration = QVector<QwtPlotCurve *>(2);
	yData = QVector<QVector<double> >(2);
	for (int i = 0; i < acceleration.size(); i++) {
		acceleration[i] = new QwtPlotCurve;
		acceleration[i]->attach(ui.qwtPlot);
		yData[i] = QVector<double>();
	}
	acceleration[0]->setPen(Qt::red);
	acceleration[1]->setPen(Qt::blue);
	xData = QVector<double>();
}

BoardConsole::~BoardConsole() {
	if (stmReader)	delete stmReader;
	if (stm)		delete stm;

	for (int i = 0; i < 2; i++) {
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
	else {
		qDebug() << "Connected";
	}

	defineLogFile();
}

void BoardConsole::handleStabToggleButton() {
	stm->write("e");
	if (ui.stabToggleButton->text() == "Start") {
		ui.stabToggleButton->setText("Stop");
	}
	else {
		ui.stabToggleButton->setText("Start");
	}
}

void BoardConsole::handleCalibrButton() {
	stm->write("g");
}

void BoardConsole::handleTelemetryButtons() {
	xData.clear();
	yData[0].clear();
	yData[1].clear();
	if (ui.fullRadioButton->isChecked()) {
		stm->write("k");
	} else if (ui.axRadioButton->isChecked()) {
		stm->write("l");
	} else if (ui.ayRadioButton->isChecked()) {
		stm->write("q");
	} else if (ui.azRadioButton->isChecked()) {
		stm->write("r");
	}
}	

void BoardConsole::handleTelemetryToggleButton() {
	stm->write("h");
	if (ui.telemetryToggleButton->text() == "Start") {
		ui.telemetryToggleButton->setText("Stop");
	} else {
		ui.telemetryToggleButton->setText("Start");
	}
}

void BoardConsole::handleAccelButtons() {
	if (ui.HZ100RadioButton->isChecked()) {
		stm->write("A");
	}
	else if (ui.HZ800RadioButton->isChecked())
	{
		stm->write("B");
	}
}


void BoardConsole::handleFreshLine(QString &line) {
	QStringList numbers = line.split(' ');
	qDebug() << numbers;
	if (ui.fullRadioButton->isChecked()) {
		if (numbers.size() != 5) return;
		for (int i = 0; i < yData.size(); i++) {
			xData.append(xData.size() + 1);
			yData[i].append(numbers[i].toDouble());
			acceleration[i]->setSamples(xData, yData[i]);
		}
	}
	else if (ui.axRadioButton->isChecked()) {
		if (numbers.size() != 2) return;
		for (int i = 0; i < yData.size(); i++) {
			xData.append(xData.size() + 1);
			yData[i].append(numbers[i].toDouble());
			acceleration[i]->setSamples(xData, yData[i]);	
		}
	}
	else if (ui.ayRadioButton->isChecked()) {
		if (numbers.size() != 2) return;
		for (int i = 0; i < yData.size(); i++) {
			xData.append(xData.size() + 1);
			yData[i].append(numbers[i].toDouble());
			acceleration[i]->setSamples(xData, yData[i]);
		}
	}
	else if (ui.azRadioButton->isChecked()) {
		if (numbers.size() != 2) return;
		for (int i = 0; i < yData.size(); i++) {
			xData.append(xData.size() + 1);
			yData[i].append(numbers[i].toDouble());
			acceleration[i]->setSamples(xData, yData[i]);
		}
	}
	ui.qwtPlot->replot();
}

void BoardConsole::handleNoFilterCheckBox() {
	if (ui.noFilterCheckBox->isChecked()) {
		ui.kalmanFilterCheckBox->setEnabled(false);
		ui.averagingCheckBox->setEnabled(false);
		if (ui.kalmanFilterCheckBox->isChecked()) {
			stm->write("s");
			ui.kalmanFilterCheckBox->setChecked(false);
		}
		if (ui.averagingCheckBox->isChecked()) {
			stm->write("t");
			ui.averagingCheckBox->setChecked(false);
		}
	} else {
		ui.kalmanFilterCheckBox->setEnabled(true);
		ui.averagingCheckBox->setEnabled(true);
	}
}

void BoardConsole::handleKalmanFilterCheckBox() {
	stm->write("s");
}

void BoardConsole::handleAveragingCheckBox() {
	stm->write("t");
}