#include "boardconsole.h"
#include <QtSerialPort\qserialportinfo.h>
#include <qdir.h>
#include <qdatetime.h>
#include "commands.h"
#include <QMouseEvent>

BoardConsole::BoardConsole(QWidget *parent)
	: QWidget(parent)
{
	ui.setupUi(this);
	glwidget = NULL;

	QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
	foreach(QSerialPortInfo port, availablePorts) {
		ui.serialComboBox->addItem(port.portName());
	}
	ui.serialComboBox->setCurrentText("COM22");

	connect(ui.rotationButton, SIGNAL(clicked()), SLOT(handleRotationButton()));

	connect(ui.programButton, SIGNAL(clicked()), SLOT(handleProgramButton()));
	connect(ui.connectButton, SIGNAL(clicked()), SLOT(handleConnectButton()));
	connect(ui.stopMotorsButton, SIGNAL(clicked()), SLOT(handleStopMotorsButton()));
	connect(ui.calibrButton, SIGNAL(clicked()), SLOT(handleCalibrButton()));
	connect(ui.clearTelemetryButton, SIGNAL(clicked()), SLOT(handleClearTelemetryButton()));

	connect(ui.telemetryToggleButton, SIGNAL(clicked()), SLOT(handleTelemetryToggleButton()));

	connect(ui.saveToFileCheckBox, SIGNAL(clicked()), SLOT(handleSaveToFileCheckBox()));

	connect(ui.pButton, SIGNAL(clicked()), SLOT(handlePButton()));
	connect(ui.iButton, SIGNAL(clicked()), SLOT(handleIButton()));
	connect(ui.dButton, SIGNAL(clicked()), SLOT(handleDButton()));


	connect(ui.pwm1SpinBox, SIGNAL(valueChanged(int)), SLOT(handlePwm1SpinBox(int)));
	connect(ui.pwm2SpinBox, SIGNAL(valueChanged(int)), SLOT(handlePwm2SpinBox(int)));

	connect(ui.impulseRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.stepRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.sineRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.expRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.noResearchRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.pidRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));
	connect(ui.operatorRadioButton, SIGNAL(clicked()), SLOT(handleResearchButtons()));

	//connect(ui.maxAngleButton, SIGNAL(clicked()), SLOT(handleMaxAngleButton()));
	//connect(ui.accelDeviationButton, SIGNAL(clicked()), SLOT(handleAccelDeviationButton()));

	connect(ui.pwm1Slider, SIGNAL(valueChanged(int)), SLOT(handlePwm1Slider(int)));
	connect(ui.pwm2Slider, SIGNAL(valueChanged(int)), SLOT(handlePwm2Slider(int)));

	paramsBitMask = 0; 
	fillParamsVector();
	fillListsVector();

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
		QString dirPath = QDir::currentPath() + "/../Logs/Ordinary/";
		QDir workDir = QDir(dirPath);
		int maxLogNumber = 0;
		foreach(QString entry, workDir.entryList()) {
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
		logFileName = dirPath + tr("log") + QString::number(maxLogNumber) + tr(".txt");
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

void BoardConsole::fillParamsVector() {
	paramCheckBoxes.append(ui.accelX);
	paramCheckBoxes.append(ui.accelY);
	paramCheckBoxes.append(ui.accelZ);

	paramCheckBoxes.append(ui.gyroX);
	paramCheckBoxes.append(ui.gyroY);
	paramCheckBoxes.append(ui.gyroZ);

	paramCheckBoxes.append(ui.compassX);
	paramCheckBoxes.append(ui.compassY);
	paramCheckBoxes.append(ui.compassZ);

	paramCheckBoxes.append(ui.eulerX);
	paramCheckBoxes.append(ui.eulerY);
	paramCheckBoxes.append(ui.eulerZ);

	paramCheckBoxes.append(ui.eulerRateX);
	paramCheckBoxes.append(ui.eulerRateY);
	paramCheckBoxes.append(ui.eulerRateZ);

	paramCheckBoxes.append(ui.pwm1);
	paramCheckBoxes.append(ui.pwm2);

	paramCheckBoxes.append(ui.freq1);
	paramCheckBoxes.append(ui.freq2);

	paramCheckBoxes.append(ui.f);

	foreach (QCheckBox *paramBox, paramCheckBoxes) {
		connect(paramBox, SIGNAL(pressed()), SLOT(paramPressed()));
		connect(paramBox, SIGNAL(stateChanged(int)), SLOT(handleParamBoxStateChanged(int)));
	}

	draggedParam = NULL;
}

void BoardConsole::fillListsVector() {
	plotLists.append(ui.plot1list);
	plotLists.append(ui.plot2list);
	plotLists.append(ui.plot3list);

	foreach(QListWidget *listWidget, plotLists) {
		connect(listWidget, SIGNAL(itemChanged(QListWidgetItem *)), SLOT(handlePlotListItemChanged(QListWidgetItem *)));
	}
}

void BoardConsole::handleProgramButton() {
	stm->write(command(PROGRAMMING_MODE));
}


void BoardConsole::handleConnectButton() {
	stm = new QSerialPort(ui.serialComboBox->currentText());
	stm->setBaudRate(QSerialPort::Baud115200);
	stm->setParity(QSerialPort::EvenParity);
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
	connect(stmReader, SIGNAL(freshMessage(const Message *)), this, SLOT(handleFreshMessage(const Message *)));

	//STM_Init();
}

void BoardConsole::STM_Init() {
	stm->write(command(TURN_EVERYTHING_OFF));
}

void BoardConsole::handleRotationButton() {
	glwidget = new GLWidget();
	glwidget->show();
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

//void Message_ToByteArray(Message *message, uint8_t *a) {
//	uint8_t i = 0, crc = 0;
//	memcpy(a + 1, (uint8_t *)message, Message_Size);
//	a[0] = MESSAGE_HEADER;
//	crc = a[0];
//	for (i = 1; i < Message_Size + 1; i++) {
//		crc ^= a[i];
//	}
//	a[Message_Size + 1] = crc;
//}

uint8_t Message_FromByteArray(const QByteArray &bytes, quint32 paramsBitMask, Message *message) {
	uint8_t i = 0, crc = 0;
	uint8_t *data = (uint8_t *)bytes.data();

	crc = data[0];
	for (i = 1; i < bytes.size()-1; i++) {
		crc ^= data[i];
	}
	if ((data[0] == MESSAGE_HEADER) && (data[bytes.size()-1] == crc)) {
		int ind = 1;
		if (paramsBitMask & BIT_ACCEL_X) {
			message->accel[0] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->accel[0]);
		}
		if (paramsBitMask & BIT_ACCEL_Y) {
			message->accel[1] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->accel[1]);
		}
		if (paramsBitMask & BIT_ACCEL_Z) {
			message->accel[2] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->accel[2]);
		}
		if (paramsBitMask & BIT_GYRO_X) {
			message->gyro[0] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->gyro[0]);
		}
		if (paramsBitMask & BIT_GYRO_Y) {
			message->gyro[1] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->gyro[1]);
		}
		if (paramsBitMask & BIT_GYRO_X) {
			message->gyro[2] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->gyro[2]);
		}
		if (paramsBitMask & BIT_COMPASS_X) {
			message->compass[0] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->compass[0]);
		}
		if (paramsBitMask & BIT_COMPASS_Y) {
			message->compass[1] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->compass[1]);
		}
		if (paramsBitMask & BIT_COMPASS_Z) {
			message->compass[2] = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->compass[2]);
		}
		if (paramsBitMask & BIT_EULER_X) {
			message->euler[0] = ((float *)&data[ind])[0];
			ind += sizeof(message->euler[0]);
		}
		if (paramsBitMask & BIT_EULER_Y) {
			message->euler[1] = ((float *)&data[ind])[0];
			ind += sizeof(message->euler[1]);
		}
		if (paramsBitMask & BIT_EULER_Z) {
			message->euler[2] = ((float *)&data[ind])[0];
			ind += sizeof(message->euler[2]);
		}
		if (paramsBitMask & BIT_EULERRATE_X) {
			message->eulerRate[0] = ((float *)&data[ind])[0];
			ind += sizeof(message->eulerRate[0]);
		}
		if (paramsBitMask & BIT_EULERRATE_Y) {
			message->eulerRate[1] = ((float *)&data[ind])[0];
			ind += sizeof(message->eulerRate[1]);
		}
		if (paramsBitMask & BIT_EULERRATE_Z) {
			message->eulerRate[2] = ((float *)&data[ind])[0];
			ind += sizeof(message->eulerRate[2]);
		}
		if (paramsBitMask & BIT_PWM1) {
			message->pwm1 = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->pwm1);
		}
		if (paramsBitMask & BIT_PWM2) {
			message->pwm2 = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->pwm2);
		}
		if (paramsBitMask & BIT_FREQ1) {
			message->freq1 = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->freq1);
		}
		if (paramsBitMask & BIT_FREQ2) {
			message->freq2 = (data[ind] << 8) | data[ind + 1];
			ind += sizeof(message->freq2);
		}
		if (paramsBitMask & BIT_F) {
			message->f = ((float *)&data[ind])[0];
			ind += sizeof(message->f);
		}

		return 1;
	}
	return 0;
}

float angle = 0;
void BoardConsole::handleFreshMessage(const Message * msg) {
	qint64 time = QDateTime::currentMSecsSinceEpoch() - startTime;

	if (glwidget) {
		glwidget->setXRotation(msg->euler[0] * 16.0f);
		glwidget->setYRotation(msg->euler[1] * 16.0f);
		glwidget->setZRotation(msg->euler[2] * 16.0f);
	}
	//if (ui.angleCheckBox->isChecked()) {
	//	if (angleX.size() == maxSize) {
	//		angleX.pop_front();
	//		angleY.pop_front();
	//	}
	//	angleX.append(time);
	//	angleY.append(msg.euler[0]);
	//	plot1_curves[0]->setSamples(angleX, angleY);
	//}
	//if (ui.angVelCheckBox->isChecked()) {
	//	if (angVelX.size() == maxSize) {
	//		angVelX.pop_front();
	//		angVelY.pop_front();
	//	}
	//	angVelX.append(time);
	//	angVelY.append(msg.eulerRate[0]/*msg.angleRate[0] * 180.0 / 3.14159*/);
	//	plot1_curves[1]->setSamples(angVelX, angVelY);
	//}

	//if (ui.pwm1CheckBox->isChecked()) {
	//	if (pwm1X.size() == maxSize) {
	//		pwm1X.pop_front();
	//		pwm1Y.pop_front();
	//	}
	//	pwm1X.append(time);
	//	pwm1Y.append(msg.euler[1]);
	//	plot2_curves[0]->setSamples(pwm1X, pwm1Y);
	//}
	//if (ui.pwm2CheckBox->isChecked()) {
	//	if (pwm2X.size() == maxSize) {
	//		pwm2X.pop_front();
	//		pwm2Y.pop_front();
	//	}
	//	pwm2X.append(time);
	//	pwm2Y.append(msg.eulerRate[1]/*msg.angleRate[1] * 180.0 / 3.14159*/);
	//	plot2_curves[1]->setSamples(pwm2X, pwm2Y);
	//}

	//if (ui.count1CheckBox->isChecked()) {
	//	if (count1X.size() == maxSize) {
	//		count1X.pop_front();
	//		count1Y.pop_front();
	//	}
	//	count1X.append(time);
	//	count1Y.append(msg.euler[2]);
	//	plot3_curves[0]->setSamples(count1X, count1Y);
	//}
	//if (ui.count2CheckBox->isChecked()) {
	//	if (count2X.size() == maxSize) {
	//		count2X.pop_front();
	//		count2Y.pop_front();
	//	}
	//	count2X.append(time);
	//	count2Y.append(msg.eulerRate[2]/*msg.angleRate[2] * 180.0 / 3.14159*/);
	//	plot3_curves[1]->setSamples(count2X, count2Y);
	//}

	//ui.plot1->replot();
	//ui.plot2->replot();
	//ui.plot3->replot();
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
	else if (ui.pidRadioButton->isChecked()) {
		stm->write(command(PID));
	}
	else if (ui.operatorRadioButton->isChecked()) {
		stm->write(command(OPERATOR));
	}
}

void BoardConsole::paramPressed() {
	QCheckBox *sender = (QCheckBox *)QObject::sender();
	foreach(QCheckBox *paramBox, paramCheckBoxes) {
		if (sender == paramBox && paramBox->isChecked()) {
			draggedParam = paramBox;
			return;
		}
	}
}

void BoardConsole::handleParamBoxStateChanged(int state) {
	QCheckBox *sender = (QCheckBox *)QObject::sender();
	for (int i = 0; i < paramCheckBoxes.size(); i++) {
		QCheckBox *paramBox = paramCheckBoxes[i];
		if (sender == paramBox) {
			switch (state) {
			case Qt::Checked:
				paramsBitMask |= 1 << i;
				break;
			case Qt::Unchecked:
				paramsBitMask &= ~(1 << i);
				stopDisplayParam(paramBox->text());
				break;
			}
			stmReader->setParamsBitMask(paramsBitMask);
			QByteArray bitMaskCommand = number_command(PARAMS_BITMASK_SYMBOL, QString::number(paramsBitMask));
			stm->write(bitMaskCommand);
		}
	}
}

void BoardConsole::mouseReleaseEvent(QMouseEvent *event) {
	if (event->button() != Qt::LeftButton) { return; }
	if (!draggedParam) { return; }
	QRect listParentRect = ui.visualGroupBox->geometry();
	QPoint cursor = event->pos();
	cursor -= listParentRect.topLeft();
	foreach(QListWidget *list, plotLists) {
		QRect listRect = list->geometry();
		if (listRect.contains(cursor)) {
			QString newItemText = draggedParam->text();
			QList<QListWidgetItem *> sameItem = list->findItems(newItemText, Qt::MatchFixedString);
			if (sameItem.empty()) {
				list->addItem(newItemText);
				startDisplayParam(newItemText);
			}
			break;
		}
	}
	draggedParam = NULL;
}

void BoardConsole::stopDisplayParam(const QString &paramName) {
	foreach(QListWidget *list, plotLists) {
		for (int i = 0; i < list->count(); i++) {
			QListWidgetItem *item = list->item(i);
			if (item->text() == paramName) {
				delete item;
			}
		}
	}
}

void BoardConsole::startDisplayParam(const QString &paramName) {

}

void BoardConsole::handlePlotListItemChanged(QListWidgetItem *item) {
	QListWidget *sender = (QListWidget *)QObject::sender();
	foreach(QListWidget *list, plotLists) {
		if (sender == list) {
			if (item->text() == "") {
				stopDisplayParam("");
			} else {
				startDisplayParam(item->text());
			}
		}
	}
}
