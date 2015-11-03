#include "boardconsole.h"
#include <QtSerialPort\qserialportinfo.h>
#include <qdir.h>
#include <qdatetime.h>
#include "commands.h"

uint8_t Message_Size = 0;

BoardConsole::BoardConsole(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	Message_Size = sizeof(Message);

	QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
	foreach(QSerialPortInfo port, availablePorts) {
		ui.serialComboBox->addItem(port.portName());
	}
	ui.serialComboBox->setCurrentText("COM22");

	connect(ui.programButton, SIGNAL(clicked()), SLOT(handleProgramButton()));
	connect(ui.connectButton, SIGNAL(clicked()), SLOT(handleConnectButton()));
	connect(ui.stopMotorsButton, SIGNAL(clicked()), SLOT(handleStopMotorsButton()));
	connect(ui.calibrButton, SIGNAL(clicked()), SLOT(handleCalibrButton()));
	connect(ui.clearTelemetryButton, SIGNAL(clicked()), SLOT(handleClearTelemetryButton()));

	connect(ui.telemetryToggleButton, SIGNAL(clicked()), SLOT(handleTelemetryToggleButton()));
	connect(ui.HZ25RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));
	connect(ui.HZ50RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));
	connect(ui.HZ100RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));
	connect(ui.HZ800RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));
	connect(ui.HZ1600RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));
	connect(ui.HZ3200RadioButton, SIGNAL(clicked()), SLOT(handleAccelButtons()));

	connect(ui.gyroHZ100RadioButton, SIGNAL(clicked()), SLOT(handleGyroButtons()));
	connect(ui.gyroHZ250RadioButton, SIGNAL(clicked()), SLOT(handleGyroButtons()));
	connect(ui.gyroHZ500RadioButton, SIGNAL(clicked()), SLOT(handleGyroButtons()));
	connect(ui.gyroHZ1000RadioButton, SIGNAL(clicked()), SLOT(handleGyroButtons()));


	connect(ui.lowpassFilterCheckBox, SIGNAL(clicked()), SLOT(handleLowpassFilterCheckBox()));


	connect(ui.saveToFileCheckBox, SIGNAL(clicked()), SLOT(handleSaveToFileCheckBox()));

	connect(ui.pButton, SIGNAL(clicked()), SLOT(handlePButton()));
	connect(ui.iButton, SIGNAL(clicked()), SLOT(handleIButton()));
	connect(ui.dButton, SIGNAL(clicked()), SLOT(handleDButton()));

	connect(ui.angleCheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.angVelCheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.fCheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.pwm1CheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));
	connect(ui.pwm2CheckBox, SIGNAL(clicked()), SLOT(handleTelemetryDisplayButtons()));

	connect(ui.pwm1SpinBox, SIGNAL(valueChanged(int)), SLOT(handlePwm1SpinBox(int)));
	connect(ui.pwm2SpinBox, SIGNAL(valueChanged(int)), SLOT(handlePwm2SpinBox(int)));

	connect(ui.impulseRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.stepRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.sineRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.expRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.noResearchRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.simpleRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.pidRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.operatorRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.adjustRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));

	connect(ui.maxAngleButton, SIGNAL(clicked()), SLOT(handleMaxAngleButton()));
	connect(ui.accelDeviationButton, SIGNAL(clicked()), SLOT(handleAccelDeviationButton()));

	connect(ui.pwm1Slider, SIGNAL(valueChanged(int)), SLOT(handlePwm1Slider(int)));
	connect(ui.pwm2Slider, SIGNAL(valueChanged(int)), SLOT(handlePwm2Slider(int)));


	connect(ui.turnoffAngleButton, SIGNAL(clicked()), SLOT(handleTurnoffAngleButton()));
	connect(ui.maxVelButton, SIGNAL(clicked()), SLOT(handleMaxVelButton()));

	connect(ui.turnUselessCheckBox, SIGNAL(clicked()), SLOT(handleTurnUselessCheckBox()));
	connect(ui.gyroRecalibrationCheckBox, SIGNAL(clicked()), SLOT(handleGyroRecalibrationCheckBox()));
	connect(ui.tranquilityButton, SIGNAL(clicked()), SLOT(handleTranquilityButton()));
	connect(ui.pwmStepButton, SIGNAL(clicked()), SLOT(handlePwmStepButton()));
	connect(ui.everyNButton, SIGNAL(clicked()), SLOT(handleEveryNButton()));

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

	plot3_curves = QVector<QwtPlotCurve *>(2);
	for (int i = 0; i < plot3_curves.size(); i++) {
		plot3_curves[i] = new QwtPlotCurve;
		plot3_curves[i]->attach(ui.plot3);
	}
	plot3_curves[0]->setPen(Qt::red);
	plot3_curves[1]->setPen(Qt::blue);

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

	count1X = QVector<double>();
	count1Y = QVector<double>();

	count2X = QVector<double>();
	count2Y = QVector<double>();


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
			int logIndex = entry.indexOf("log");
			if (logIndex != -1) {
				int txtIndex = entry.indexOf(".txt");
				int logNumber = entry.mid(logIndex + 3, txtIndex - 3).toInt();
				if (logNumber > maxLogNumber) {
					maxLogNumber = logNumber;
				}
			}
		}
		maxLogNumber++;
		//QString maxAngle = ui.maxAngleLineEdit->text();
		//QString div = ui.pwm2SpinBox->value() == 2000 ? "" : "div";
		logFileName = tr("log") + QString::number(maxLogNumber) + tr(".txt");
	}
	return logFileName;
	
	
}

QByteArray BoardConsole::command(const char c) {
	return QString(c).toLatin1();
}

QByteArray BoardConsole::number_command(const char c, QString number) {
	QString str = QString(c) + number + QString(NUMBER_END);
	return str.toLatin1();
}

QByteArray BoardConsole::double_number_command(const char c, QString num1, QString num2) {
	QString str = QString(c) + num1 + QString(NUMBER_END) + num2 + QString(NUMBER_END);
	return str.toLatin1();
}

void BoardConsole::handleProgramButton() {
	stm->write(command(PROGRAMMING_MODE));
}


void BoardConsole::handleConnectButton() {
	stm = new QSerialPort(ui.serialComboBox->currentText());
	stm->setBaudRate(QSerialPort::Baud115200);
	//stm->setParity(QSerialPort::EvenParity);
	stm->setStopBits(QSerialPort::OneStop);
	//stm->setFlowControl(QSerialPort::NoFlowControl);
	if (!stm->open(QIODevice::ReadWrite)) {
		qDebug() << QObject::tr("Failed to open port %1, error: %2").arg(stm->portName()).arg(stm->errorString()) << endl;
	}
	else {
		qDebug() << "Connected";
	}

	QString logFileName  = defineLogFile();
	stmReader = new SerialPortReader(stm, logFileName);
	connect(stmReader, SIGNAL(freshLine(QString &)), SLOT(handleFreshLine(QString &)));
	connect(stmReader, SIGNAL(freshMessage(Message)), SLOT(handleFreshMessage(Message)));

	//STM_Init();
}

void BoardConsole::STM_Init() {
	stm->write(command(TURN_EVERYTHING_OFF));
	stm->write(number_command(ACCEL_DEVIATION, "0"));
}

void BoardConsole::handleStopMotorsButton() {	
	ui.noResearchRadioButton->setChecked(true);
	stm->write(command(STOP_MOTORS));
	handleResearchButtons();
}

void BoardConsole::handleCalibrButton() {
	stm->write(command(CALIBRATION));
	ui.noResearchRadioButton->setChecked(true);
}

void BoardConsole::handleClearTelemetryButton() {
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

	count1X.clear();
	count1Y.clear();

	count2X.clear();
	count2Y.clear();

	plot1_curves[0]->setSamples(angleX, angleY);
	plot1_curves[1]->setSamples(angVelX, angVelY);

	plot2_curves[0]->setSamples(fX, fY);
	plot2_curves[1]->setSamples(pwm1X, pwm1Y);
	plot2_curves[2]->setSamples(pwm2X, pwm2Y);

	plot3_curves[0]->setSamples(count1X, count1Y);
	plot3_curves[1]->setSamples(count2X, count2Y);

	ui.plot1->replot();
	ui.plot2->replot();
	ui.plot3->replot();
}	

void BoardConsole::handleTelemetryToggleButton() {
	stm->write(command(TELEMETRY));
	if (ui.telemetryToggleButton->text() == "Start") {
		ui.telemetryToggleButton->setText("Stop");
	} else {
		ui.telemetryToggleButton->setText("Start");
	}
}

void BoardConsole::handleAccelButtons() {
	if (ui.HZ25RadioButton->isChecked()) {
		stm->write(command(ACCEL_FREQ_HZ25));
	}
	else if (ui.HZ50RadioButton->isChecked()) {
		stm->write(command(ACCEL_FREQ_HZ50));
	}
	else if (ui.HZ100RadioButton->isChecked()) {
		stm->write(command(ACCEL_FREQ_HZ100));
	}
	else if (ui.HZ800RadioButton->isChecked())
	{
		stm->write(command(ACCEL_FREQ_HZ800));
	}
	else if (ui.HZ1600RadioButton->isChecked()) {
		stm->write(command(ACCEL_FREQ_HZ1600));
	}
	else if (ui.HZ3200RadioButton->isChecked()) {
		stm->write(command(ACCEL_FREQ_HZ3200));
	}
}

void BoardConsole::handleGyroButtons() {
	if (ui.gyroHZ100RadioButton->isChecked()) {
		stm->write(command(GYRO_FREQ_HZ100));
	}
	else if (ui.gyroHZ250RadioButton->isChecked()) {
		stm->write(command(GYRO_FREQ_HZ250));
	}
	else if (ui.gyroHZ500RadioButton->isChecked()) {
		stm->write(command(GYRO_FREQ_HZ500));
	}
	else if (ui.gyroHZ1000RadioButton->isChecked()) {
		stm->write(command(GYRO_FREQ_HZ1000));
	}
}
void Message_ToByteArray(Message *message, uint8_t *a) {
	uint8_t i = 0, crc = 0;
	memcpy(a + 1, (uint8_t *)message, Message_Size);
	a[0] = MESSAGE_HEADER;
	crc = a[0];
	for (i = 1; i < Message_Size + 1; i++) {
		crc ^= a[i];
	}
	a[Message_Size + 1] = crc;
}

uint8_t Message_FromByteArray(uint8_t *a, uint8_t n, Message *message) {
	uint8_t i = 0, crc = 0;

	crc = a[0];
	for (i = 1; i < Message_Size + 1; i++) {
		crc ^= a[i];
	}
	if ((a[0] == MESSAGE_HEADER) && (a[Message_Size + 1] == crc)) {
		memcpy((uint8_t *)message, a + 1, Message_Size);
		return 1;
	}
	return 0;
}

float angle = 0;
void BoardConsole::handleFreshMessage(Message msg) {
	/*float arate = msg.ars3_z / 80.0;
	angle += arate * 0.01;*/
	/*qDebug() << msg.ars1_x << ' ' << msg.ars1_y << ' ' << msg.ars1_t << ' '
		<< msg.ars2_x << ' ' << msg.ars2_y << ' ' << msg.ars2_t << ' ' << msg.ars3_z << ' '
		<< msg.accel_x << ' ' << msg.accel_y << ' ' << msg.accel_z << endl;*/
	qDebug() << msg.roll << msg.pitch << endl;

	/*qint64 time = QDateTime::currentMSecsSinceEpoch() - startTime;

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
		angVelY.append(arate);
		plot1_curves[1]->setSamples(angVelX, angVelY);

		ui.plot1->replot();
	}*/
}

void BoardConsole::handleFreshLine(QString &line) {
	if (firstMeasurement) {
		startTime = QDateTime::currentMSecsSinceEpoch();
		firstMeasurement = false;
	}

	QStringList numbers = line.split(' ');
	//qDebug() << numbers;

	if (numbers.size() < 11) return;

	double angle = numbers[0].toDouble() * 180 / 3.14159;
	double angularVelocity = numbers[1].toDouble() * 180 / 3.14159;
	double F = numbers[2].toDouble();
	double pwm1 = numbers[6].toDouble();
	double pwm2 = numbers[7].toDouble();
	double count1 = 0; 
	double count2 = 0;
	if (numbers[8].toDouble() != 0) {
		count1 = 14e6 / numbers[8].toDouble();
	}
	if (numbers[9].toDouble()) {
		count2 = 14e6 / numbers[9].toDouble();
	}

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

	if (ui.count1CheckBox->isChecked()) {
		if (count1X.size() == maxSize) {
			count1X.pop_front();
			count1Y.pop_front();
		}
		count1X.append(time);
		count1Y.append(count1);
		plot3_curves[0]->setSamples(count1X, count1Y);
	}
	if (ui.count2CheckBox->isChecked()) {
		if (count2X.size() == maxSize) {
			count2X.pop_front();
			count2Y.pop_front();
		}
		count2X.append(time);
		count2Y.append(count2);
		plot3_curves[1]->setSamples(count2X, count2Y);
	}


	ui.plot1->replot();
	ui.plot2->replot();
	ui.plot3->replot();
}


void BoardConsole::handleLowpassFilterCheckBox() {
	stm->write(command(LOWPASS));
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

void BoardConsole::handlePButton() {
	QString kStr = ui.pLineEdit->text();
	if (!kStr.isEmpty()) {
		stm->write(number_command(KP, kStr));
	}
}

void BoardConsole::handleIButton() {
	QString kStr = ui.iLineEdit->text();
	if (!kStr.isEmpty()) {
		stm->write(number_command(KI, kStr));
	}
}

void BoardConsole::handleDButton() {
	QString kStr = ui.dLineEdit->text();
	if (!kStr.isEmpty()) {
		stm->write(number_command(KD, kStr));
	}
}

void BoardConsole::handleTelemetryDisplayButtons() {

}

void BoardConsole::handlePwm1SpinBox(int newval) {
	stm->write(number_command(MIN_PWM, QString::number(newval)));
}

void BoardConsole::handlePwm2SpinBox(int newval) {
	//handleSaveToFileCheckBox();
	//handleSaveToFileCheckBox();
	stm->write(number_command(MAX_PWM, QString::number(newval)));
}

void BoardConsole::handlePwm1Slider(int newval) {
	stm->write(number_command(PWM1, QString::number(newval)));
	ui.pwm1Label->setText(QString::number(newval));
}

void BoardConsole::handlePwm2Slider(int newval) {
	stm->write(number_command(PWM2, QString::number(newval)));
	ui.pwm2Label->setText(QString::number(newval));
}

void BoardConsole::handleResearchButtons() {
	QString ampl = ui.researchAmplLineEdit->text();
	QString freq = ui.researchFreqLineEdit->text();
	if (ui.impulseRadioButton->isChecked()) {
		stm->write(command(IMPULSE));
	}
	else if (ui.stepRadioButton->isChecked()) {
		stm->write(command(STEP));
	}
	else if (ui.sineRadioButton->isChecked()) {
		stm->write(double_number_command(SINE, ampl, freq));
	} 
	else if (ui.expRadioButton->isChecked()) {
		stm->write(double_number_command(EXP, ampl, freq));
	}
	else if (ui.noResearchRadioButton->isChecked()) {
		stm->write(command(NO_RESEARCH_SYMBOL));
	}
	else if (ui.simpleRadioButton->isChecked()) {
		stm->write(command(SIMPLE));
	}
	else if (ui.pidRadioButton->isChecked()) {
		stm->write(command(PID));
	}
	else if (ui.operatorRadioButton->isChecked()) {
		stm->write(command(OPERATOR));
	}
	else if (ui.adjustRadioButton->isChecked()) {
		stm->write(command(ADJUST));
	}
}

void BoardConsole::handleMaxAngleButton() {
	//handleSaveToFileCheckBox();
	//handleSaveToFileCheckBox();
	QString maxAngleStr = ui.maxAngleLineEdit->text();
	double inRadians = maxAngleStr.toDouble() * 3.14159 / 180;
	stm->write(number_command(MAX_ANGLE, QString::number(inRadians)));
}

void BoardConsole::handleAccelDeviationButton() {
	QString dfStr = ui.accelDeviationLineEdit->text();
	double deviation = tan(dfStr.toDouble()) * tan(dfStr.toDouble());
	stm->write(number_command(ACCEL_DEVIATION, QString::number(deviation)));
}

void BoardConsole::handleTurnoffAngleButton() {
	QString boundaryAngleStr = ui.turnoffAngleLineEdit->text();
	double inRadians = boundaryAngleStr.toDouble() * 3.14159 / 180;
	stm->write(number_command(BOUNDARY_ANGLE, QString::number(inRadians)));
}

void BoardConsole::handleMaxVelButton(){
	QString maxVel = ui.maxVelLineEdit->text();
	double inRadians = maxVel.toDouble() * 3.14159 / 180;
	stm->write(number_command(MAX_ANGVEL, QString::number(inRadians)));
}

void BoardConsole::handleTurnUselessCheckBox() {
	stm->write(command(TURN_USELESS));
}

void BoardConsole::handleGyroRecalibrationCheckBox() {
	stm->write(command(GYRO_RECALIBRATION));
}

void BoardConsole::handleTranquilityButton() {
	QString time = ui.tranquilityLineEdit->text();
	stm->write(number_command(TRANQUILITY_TIME, time));
}

void BoardConsole::handlePwmStepButton() {
	QString step = ui.pwmStepLineEdit->text();
	stm->write(number_command(PWM_STEP, step));
}

void BoardConsole::handleEveryNButton() {
	QString n = ui.everyNLineEdit->text();
	stm->write(number_command(EVERY_N, n));
}