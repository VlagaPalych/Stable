#include "boardconsole.h"
#include <QtSerialPort\qserialportinfo.h>
#include <qdir.h>
#include <qdatetime.h>

BoardConsole::BoardConsole(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);

	QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
	foreach(QSerialPortInfo port, availablePorts) {
		ui.serialComboBox->addItem(port.portName());
	}
	ui.serialComboBox->setCurrentText("COM18");

	connect(ui.connectButton, SIGNAL(clicked()), SLOT(handleConnectButton()));
	connect(ui.stabToggleButton, SIGNAL(clicked()), SLOT(handleStabToggleButton()));
	connect(ui.calibrButton, SIGNAL(clicked()), SLOT(handleCalibrButton()));
	connect(ui.fullRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.moveRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.axRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.ayRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.azRadioButton, SIGNAL(clicked()), SLOT(handleTelemetryButtons()));
	connect(ui.telemetryToggleButton, SIGNAL(clicked()), SLOT(handleTelemetryToggleButton()));
	connect(ui.HZ100RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));
	connect(ui.HZ800RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));

	connect(ui.noFilterCheckBox, SIGNAL(clicked()), SLOT(handleNoFilterCheckBox()));
	connect(ui.kalmanFilterCheckBox, SIGNAL(clicked()), SLOT(handleKalmanFilterCheckBox()));
	connect(ui.averagingCheckBox, SIGNAL(clicked()), SLOT(handleAveragingCheckBox()));

	connect(ui.saveToFileCheckBox, SIGNAL(clicked()), SLOT(handleSaveToFileCheckBox()));

	connect(ui.k1Button, SIGNAL(clicked()), SLOT(handleK1Button()));
	connect(ui.k2Button, SIGNAL(clicked()), SLOT(handleK2Button()));

	connect(ui.angleCheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.angVelCheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.fCheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.pwm1CheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.pwm2CheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));

	connect(ui.pwm1SpinBox, SIGNAL(valueChanged(int)), SLOT(handlePwm1SpinBox(int)));
	connect(ui.pwm2SpinBox, SIGNAL(valueChanged(int)), SLOT(handlePwm2SpinBox(int)));

	stm = NULL;
	stmReader = NULL;

	plot1_curves = QVector<QwtPlotCurve *>(2);
	for (int i = 0; i < plot1_curves.size(); i++) {
		plot1_curves[i] = new QwtPlotCurve;
		plot1_curves[i]->attach(ui.plot1);
	}
	plot1_curves[0]->setPen(Qt::red);
	plot1_curves[1]->setPen(Qt::blue);

	plot2_curves = QVector<QwtPlotCurve *>(3);
	for (int i = 0; i < plot2_curves.size(); i++) {
		plot2_curves[i] = new QwtPlotCurve;
		plot2_curves[i]->attach(ui.plot2);
	}
	plot2_curves[0]->setPen(Qt::red);
	plot2_curves[1]->setPen(Qt::blue);
	plot2_curves[2]->setPen(Qt::green);

	angleX = QVector<double>();
	angleY = QVector<double>();

	angVelX = QVector<double>();
	angVelY = QVector<double>();

	fX = QVector<double>();
	fY = QVector<double>();

	pwm1X = QVector<double>();
	pwm1Y = QVector<double>();

	pwm2X = QVector<double>();
	pwm2Y = QVector<double>();


	firstMeasurement = true;
	maxSize = 1000;
}

BoardConsole::~BoardConsole() {
	if (stmReader)	delete stmReader;
	if (stm)		delete stm;

	for (int i = 0; i < 2; i++) {
		delete plot1_curves[i];
	}
}

QString BoardConsole::defineLogFile() {
	QString logFileName = "";
	if (ui.saveToFileCheckBox->isChecked()) {
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
		logFileName = tr("log") + QString::number(maxLogNumber) + tr(".txt");
	}
	return logFileName;
	
	
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

	QString logFileName  = defineLogFile();
	stmReader = new SerialPortReader(stm, logFileName);
	connect(stmReader, SIGNAL(freshLine(QString &)), SLOT(handleFreshLine(QString &)));

	STM_Init();
}

void BoardConsole::STM_Init() {
	stm->write("a");
	stm->write("m1250b");
	stm->write("o2.0b");
	stm->write("p0.01b");
}

void BoardConsole::handleStabToggleButton() {	
	if (ui.stabToggleButton->text() == "Start") {
		ui.stabToggleButton->setText("Stop");
		ui.saveToFileCheckBox->setChecked(true);
		if (ui.impulseCheckBox->isChecked()) {
			stm->write("C");
		}
	}
	else {
		ui.stabToggleButton->setText("Start");
		ui.saveToFileCheckBox->setChecked(false);
	}
	handleSaveToFileCheckBox();
	stm->write("e");
}

void BoardConsole::handleCalibrButton() {
	stm->write("d");
	ui.stabToggleButton->setText("Start");
}

void BoardConsole::handleTelemetryButtons() {
	firstMeasurement = true;
	
	angleX.clear();
	angleY.clear();

	angVelX.clear();
	angVelY.clear();

	fX.clear();
	fY.clear();

	pwm1X.clear();
	pwm1Y.clear();

	pwm2X.clear();
	pwm2Y.clear();

	plot1_curves[0]->setSamples(angleX, angleY);
	plot1_curves[1]->setSamples(angVelX, angVelY);
	plot2_curves[0]->setSamples(fX, fY);
	plot2_curves[1]->setSamples(pwm1X, pwm1Y);
	plot2_curves[2]->setSamples(pwm2X, pwm2Y);

	ui.plot1->replot();
	ui.plot2->replot();

	if (ui.fullRadioButton->isChecked()) {
		stm->write("i");
	} else if (ui.moveRadioButton->isChecked()) {
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
	if (firstMeasurement) {
		startTime = QDateTime::currentMSecsSinceEpoch();
		firstMeasurement = false;
	}

	QStringList numbers = line.split(' ');
	qDebug() << numbers;
	if (ui.fullRadioButton->isChecked()) {
		if (numbers.size() != 10) return;

		double angle = numbers[0].toDouble();
		double angularVelocity = numbers[1].toDouble();
		double F = numbers[2].toDouble();
		double pwm1 = numbers[6].toDouble();
		double pwm2 = numbers[7].toDouble();

		qint64 time = QDateTime::currentMSecsSinceEpoch() - startTime;

		if (ui.angleCheckBox->isChecked()) {
			if (angleX.size() == maxSize) {
				angleX.pop_front();
				angleY.pop_front();
			}
			angleX.append(time);
			angleY.append(angle);
			plot1_curves[0]->setSamples(angleX, angleY);
		} 
		if (ui.angVelCheckBox->isChecked()) {
			if (angVelX.size() == maxSize) {
				angVelX.pop_front();
				angVelY.pop_front();
			}
			angVelX.append(time);
			angVelY.append(angularVelocity);
			plot1_curves[1]->setSamples(angVelX, angVelY);
		}
		if (ui.fCheckBox->isChecked()) {
			if (fX.size() == maxSize) {
				fX.pop_front();
				fY.pop_front();
			}
			fX.append(time);
			fY.append(F);
			plot2_curves[0]->setSamples(fX, fY);
		}
		if (ui.pwm1CheckBox->isChecked()) {
			if (pwm1X.size() == maxSize) {
				pwm1X.pop_front();
				pwm1Y.pop_front();
			}
			pwm1X.append(time);
			pwm1Y.append(pwm1);
			plot2_curves[1]->setSamples(pwm1X, pwm1Y);
		}
		if (ui.pwm2CheckBox->isChecked()) {
			if (pwm2X.size() == maxSize) {
				pwm2X.pop_front();
				pwm2Y.pop_front();
			}
			pwm2X.append(time);
			pwm2Y.append(pwm2);
			plot2_curves[2]->setSamples(pwm2X, pwm2Y);
		}

		/*for (int i = 0; i < plot1_yData.size(); i++) {
			plot1_xData.append(plot1_xData.size() + 1);
			plot1_yData[i].append(numbers[i].toDouble());
			plot1_curves[i]->setSamples(plot1_xData, plot1_yData[i]);
		}*/
	}
	/*else if (ui.axRadioButton->isChecked()) {
		if (numbers.size() != 2) return;
		for (int i = 0; i < plot1_yData.size(); i++) {
			plot1_xData.append(plot1_xData.size() + 1);
			plot1_yData[i].append(numbers[i].toDouble());
			plot1_curves[i]->setSamples(plot1_xData, plot1_yData[i]);
		}
	}
	else if (ui.ayRadioButton->isChecked()) {
		if (numbers.size() != 2) return;
		for (int i = 0; i < plot1_yData.size(); i++) {
			plot1_xData.append(plot1_xData.size() + 1);
			plot1_yData[i].append(numbers[i].toDouble());
			plot1_curves[i]->setSamples(plot1_xData, plot1_yData[i]);
		}
	}
	else if (ui.azRadioButton->isChecked()) {
		if (numbers.size() != 2) return;
		for (int i = 0; i < plot1_yData.size(); i++) {
			plot1_xData.append(plot1_xData.size() + 1);
			plot1_yData[i].append(numbers[i].toDouble());
			plot1_curves[i]->setSamples(plot1_xData, plot1_yData[i]);
		}
	}*/
	ui.plot1->replot();
	ui.plot2->replot();
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

void BoardConsole::handleSaveToFileCheckBox() {
	QString logFileName = "";
	if (ui.saveToFileCheckBox->isChecked()) {
		 logFileName = defineLogFile();
	}
	if (stmReader) {
		stmReader->setFile(logFileName);
	}
}

void BoardConsole::handleK1Button() {
	QString kStr = ui.k1LineEdit->text();
	if (!kStr.isEmpty()) {
		stm->write(tr("o%1b").arg(kStr).toStdString().c_str());
	}
}

void BoardConsole::handleK2Button() {
	QString kStr = ui.k2LineEdit->text();
	if (!kStr.isEmpty()) {
		stm->write(tr("p%1b").arg(kStr).toStdString().c_str());
	}
}

void BoardConsole::handleTelemetryDisplayButtons() {

}

void BoardConsole::handlePwm1SpinBox(int newval) {
	stm->write(tr("m%1b").arg(newval).toStdString().c_str());
}

void BoardConsole::handlePwm2SpinBox(int newval) {
	stm->write(tr("n%1b").arg(newval).toStdString().c_str());
}