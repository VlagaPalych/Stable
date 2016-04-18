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
	fillParamsVector();
	fillListsVector();
	
	glwidget = NULL;
	stm = NULL;
	stmReader = NULL;
	QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
	foreach(QSerialPortInfo port, availablePorts) {
		ui.serialComboBox->addItem(port.portName());
	}

	readSettings();
	initPlots();


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

	firstMeasurement = true;
	maxSize = 1000;
}

BoardConsole::~BoardConsole() {
	if (stmReader)	delete stmReader;
	if (stm)		delete stm;

	writeSettings();
}

void BoardConsole::initPlots() {
	plots.append(ui.plot1);
	plots.append(ui.plot2);
	plots.append(ui.plot3);

	plotCurves = QVector<QVector<QwtPlotCurve *> >(plots.size());
	plotCurveY = QVector<QVector<QVector<double> > >(plots.size());
	for (int i = 0; i < plots.size(); i++) {
		plotCurveY[i] = QVector<QVector<double> >();
	}

	paramValues = QMap<QString, QVector<double> >();
	foreach(QCheckBox *paramBox, paramCheckBoxes) {
		paramValues[paramBox->text()] = QVector<double>();
	}

	curveColors.append(Qt::red);
	curveColors.append(Qt::blue);
	curveColors.append(Qt::green);
	curveColors.append(Qt::black);

	for (int i = 0; i < plotLists.size(); i++) {
		for (int j = 0; j < plotLists[i]->count(); j++) {
			startDisplayParam(i, plotLists[i]->item(j)->text());
		}
	}
}

void BoardConsole::readSettings() {
	settings = new QSettings("boardconsole.ini", QSettings::IniFormat, this);

	ui.serialComboBox->setCurrentText(settings->value("COM-Port").toString());
	paramsBitMask = settings->value("ParamsMask").toInt();
	for (int i = 0; i < paramCheckBoxes.size(); i++) {
		if (paramsBitMask & (1 << i)) {
			paramCheckBoxes[i]->setCheckState(Qt::Checked);
		}
	}
	ui.pwm1SpinBox->setValue(settings->value("MinPwm").toInt());
	ui.pwm2SpinBox->setValue(settings->value("MaxPwm").toInt());
	ui.researchAmplLineEdit->setText(settings->value("ResearchAmpl").toString());
	ui.researchFreqLineEdit->setText(settings->value("ResearchFreq").toString());
	ui.pLineEdit->setText(settings->value("Kp").toString());
	ui.iLineEdit->setText(settings->value("Ki").toString());
	ui.dLineEdit->setText(settings->value("Kd").toString());

	int orientMethod = settings->value("OrientMethod").toInt();
	switch (orientMethod) {
	case 0:
		ui.mpl->setChecked(true);
		break;
	case 1:
		ui.dmp->setChecked(true);
		break;
	case 2:
		ui.mine->setChecked(true);
		break;
	}

	int plotListsSize = settings->beginReadArray("PlotLists");
	for (int i = 0; i < plotListsSize; i++) {
		settings->setArrayIndex(i);
		QString arrayName = QString("Plot%1").arg(i + 1);
		int plotISize = settings->beginReadArray(arrayName);
		for (int j = 0; j < plotISize; j++) {
			settings->setArrayIndex(j);
			QString elemName = QString("Param%1").arg(j + 1);
			QString listItemText = settings->value(elemName).toString();
			plotLists[i]->addItem(listItemText);
		}
		settings->endArray();
	}
	settings->endArray();
}

void BoardConsole::writeSettings() {
	settings->setValue("COM-Port", ui.serialComboBox->currentText());
	settings->setValue("ParamsMask", paramsBitMask);
	settings->setValue("MinPwm", ui.pwm1SpinBox->value());
	settings->setValue("MaxPwm", ui.pwm2SpinBox->value());
	settings->setValue("ResearchAmpl", ui.researchAmplLineEdit->text());
	settings->setValue("ResearchFreq", ui.researchFreqLineEdit->text());
	settings->setValue("Kp", ui.pLineEdit->text());
	settings->setValue("Ki", ui.iLineEdit->text());
	settings->setValue("Kd", ui.dLineEdit->text());

	if (ui.mpl->isChecked()) {
		settings->setValue("OrientMethod", 0);
	} else if (ui.dmp->isChecked()) {
		settings->setValue("OrientMethod", 1);
	} else if (ui.mine->isChecked()) {
		settings->setValue("OrientMethod", 2);
	}

	settings->beginWriteArray("PlotLists");
	for (int i = 0; i < plotLists.size(); i++) {
		settings->setArrayIndex(i);
		QString arrayName = QString("Plot%1").arg(i + 1);
		settings->beginWriteArray(arrayName);
		for (int j = 0; j < plotLists[i]->count(); j++) {
			settings->setArrayIndex(j);
			QString elemName = QString("Param%1").arg(j + 1);
			settings->setValue(elemName, plotLists[i]->item(j)->text());
		}
		settings->endArray();
	}
	settings->endArray();
	delete settings;
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

	paramCheckBoxes.append(ui.mplEulerX);
	paramCheckBoxes.append(ui.mplEulerY);
	paramCheckBoxes.append(ui.mplEulerZ);

	paramCheckBoxes.append(ui.dmpEulerX);
	paramCheckBoxes.append(ui.dmpEulerY);
	paramCheckBoxes.append(ui.dmpEulerZ);

	paramCheckBoxes.append(ui.mineEulerX);
	paramCheckBoxes.append(ui.mineEulerY);
	paramCheckBoxes.append(ui.mineEulerZ);

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
	new QShortcut(QKeySequence(Qt::Key_Delete), this, SLOT(deleteListWidgetItems()));
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
		QString logFileName = defineLogFile();
		stmReader = new SerialPortReader(stm, logFileName);
		connect(stmReader, SIGNAL(freshMessage(const Message *)), this, SLOT(handleFreshMessage(const Message *)));

		STM_Init();
		enableUI(true);
	}	
}

void BoardConsole::enableUI(bool enable) {
	ui.programButton->setEnabled(enable);
	ui.telemetryToggleButton->setEnabled(enable);
	ui.mpl->setEnabled(enable);
	ui.dmp->setEnabled(enable);
	ui.mine->setEnabled(enable);
	ui.pButton->setEnabled(enable);
	ui.iButton->setEnabled(enable);
	ui.dButton->setEnabled(enable);
	ui.stopMotorsButton->setEnabled(enable);
	ui.calibrButton->setEnabled(enable);
	ui.pwm1SpinBox->setEnabled(enable);
	ui.pwm1SpinBox->setEnabled(enable);
	ui.pwm2Slider->setEnabled(enable);
	ui.pwm2Slider->setEnabled(enable);
	ui.noResearchRadioButton->setEnabled(enable);
	ui.operatorRadioButton->setEnabled(enable);
	ui.pidRadioButton->setEnabled(enable);
	ui.impulseRadioButton->setEnabled(enable);
	ui.stepRadioButton->setEnabled(enable);
	ui.sineRadioButton->setEnabled(enable);
	ui.expRadioButton->setEnabled(enable);
}

void BoardConsole::STM_Init() {
	//stm->write(command(TURN_EVERYTHING_OFF));
	applyParamsMask();
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

	plotCurveX.clear();
	foreach(QString param, paramValues.keys()) {
		paramValues[param].clear();
	}
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

void fillFloat(float *pFloat, uint8_t *fourBytes) {
	for (int i = 0; i < 4; i++) {
		((uint8_t *)pFloat)[i] = fourBytes[i];
	}
}

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
			fillFloat(&message->accel[0], &data[ind]);
			ind += sizeof(message->accel[0]);
		}
		if (paramsBitMask & BIT_ACCEL_Y) {
			fillFloat(&message->accel[1], &data[ind]);
			ind += sizeof(message->accel[1]);
		}
		if (paramsBitMask & BIT_ACCEL_Z) {
			fillFloat(&message->accel[2], &data[ind]);
			ind += sizeof(message->accel[2]);
		}
		if (paramsBitMask & BIT_GYRO_X) {
			fillFloat(&message->gyro[0], &data[ind]);
			ind += sizeof(message->gyro[0]);
		}
		if (paramsBitMask & BIT_GYRO_Y) {
			fillFloat(&message->gyro[1], &data[ind]);
			ind += sizeof(message->gyro[1]);
		}
		if (paramsBitMask & BIT_GYRO_Z) {
			fillFloat(&message->gyro[2], &data[ind]);
			ind += sizeof(message->gyro[2]);
		}
		if (paramsBitMask & BIT_COMPASS_X) {
			fillFloat(&message->compass[0], &data[ind]);
			ind += sizeof(message->compass[0]);
		}
		if (paramsBitMask & BIT_COMPASS_Y) {
			fillFloat(&message->compass[1], &data[ind]);
			ind += sizeof(message->compass[1]);
		}
		if (paramsBitMask & BIT_COMPASS_Z) {
			fillFloat(&message->compass[2], &data[ind]);
			ind += sizeof(message->compass[2]);
		}
		if (paramsBitMask & BIT_MPL_EULER_X) {
			fillFloat(&message->mpl_euler[0], &data[ind]);
			ind += sizeof(message->mpl_euler[0]);
		}
		if (paramsBitMask & BIT_MPL_EULER_Y) {
			fillFloat(&message->mpl_euler[1], &data[ind]);
			ind += sizeof(message->mpl_euler[1]);
		}
		if (paramsBitMask & BIT_MPL_EULER_Z) {
			fillFloat(&message->mpl_euler[2], &data[ind]);
			ind += sizeof(message->mpl_euler[2]);
		}
		if (paramsBitMask & BIT_DMP_EULER_X) {
			fillFloat(&message->dmp_euler[0], &data[ind]);
			ind += sizeof(message->dmp_euler[0]);
		}
		if (paramsBitMask & BIT_DMP_EULER_Y) {
			fillFloat(&message->dmp_euler[1], &data[ind]);
			ind += sizeof(message->dmp_euler[1]);
		}
		if (paramsBitMask & BIT_DMP_EULER_Z) {
			fillFloat(&message->dmp_euler[2], &data[ind]);
			ind += sizeof(message->dmp_euler[2]);
		}
		if (paramsBitMask & BIT_MINE_EULER_X) {
			fillFloat(&message->mine_euler[0], &data[ind]);
			ind += sizeof(message->mine_euler[0]);
		}
		if (paramsBitMask & BIT_MINE_EULER_Y) {
			fillFloat(&message->mine_euler[1], &data[ind]);
			ind += sizeof(message->mine_euler[1]);
		}
		if (paramsBitMask & BIT_MINE_EULER_Z) {

			fillFloat(&message->mine_euler[2], &data[ind]);
			ind += sizeof(message->mine_euler[2]);
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
			fillFloat(&message->f, &data[ind]);
			ind += sizeof(message->f);
		}

		return 1;
	}
	return 0;
}

float angle = 0;

void BoardConsole::appendFreshData(const Message * msg) {
	foreach(QString paramName, paramValues.keys()) {
		if (paramValues[paramName].size() >= maxSize) {
			paramValues[paramName].pop_front();
		}
	}

	if (paramsBitMask & BIT_ACCEL_X) {
		paramValues[ui.accelX->text()].append(msg->accel[0]);
	}
	if (paramsBitMask & BIT_ACCEL_Y) {
		paramValues[ui.accelY->text()].append(msg->accel[1]);
	}
	if (paramsBitMask & BIT_ACCEL_Z) {
		paramValues[ui.accelZ->text()].append(msg->accel[2]);
	}
	if (paramsBitMask & BIT_GYRO_X) {
		paramValues[ui.gyroX->text()].append(msg->gyro[0]);
	}
	if (paramsBitMask & BIT_GYRO_Y) {
		paramValues[ui.gyroY->text()].append(msg->gyro[1]);
	}
	if (paramsBitMask & BIT_GYRO_Z) {
		paramValues[ui.gyroZ->text()].append(msg->gyro[2]);
	}
	if (paramsBitMask & BIT_COMPASS_X) {
		paramValues[ui.compassX->text()].append(msg->compass[0]);
	}
	if (paramsBitMask & BIT_COMPASS_Y) {
		paramValues[ui.compassY->text()].append(msg->compass[1]);
	}
	if (paramsBitMask & BIT_COMPASS_Z) {
		paramValues[ui.compassZ->text()].append(msg->compass[2]);
	}
	if (paramsBitMask & BIT_MPL_EULER_X) {
		paramValues[ui.mplEulerX->text()].append(msg->mpl_euler[0]);
	}
	if (paramsBitMask & BIT_MPL_EULER_Y) {
		paramValues[ui.mplEulerY->text()].append(msg->mpl_euler[1]);
	}
	if (paramsBitMask & BIT_MPL_EULER_Z) {
		paramValues[ui.mplEulerZ->text()].append(msg->mpl_euler[2]);
	}
	if (paramsBitMask & BIT_DMP_EULER_X) {
		paramValues[ui.dmpEulerX->text()].append(msg->dmp_euler[0]);
	}
	if (paramsBitMask & BIT_DMP_EULER_Y) {
		paramValues[ui.dmpEulerY->text()].append(msg->dmp_euler[1]);
	}
	if (paramsBitMask & BIT_DMP_EULER_Z) {
		paramValues[ui.dmpEulerZ->text()].append(msg->dmp_euler[2]);
	}
	if (paramsBitMask & BIT_MINE_EULER_X) {
		paramValues[ui.mineEulerX->text()].append(msg->mine_euler[0]);
	}
	if (paramsBitMask & BIT_MINE_EULER_Y) {
		paramValues[ui.mineEulerY->text()].append(msg->mine_euler[1]);
	}
	if (paramsBitMask & BIT_MINE_EULER_Z) {
		paramValues[ui.mineEulerZ->text()].append(msg->mine_euler[2]);
	}
	if (paramsBitMask & BIT_PWM1) {
		paramValues[ui.pwm1->text()].append(msg->pwm1);
	}
	if (paramsBitMask & BIT_PWM2) {
		paramValues[ui.pwm2->text()].append(msg->pwm2);
	}
	if (paramsBitMask & BIT_FREQ1) {
		paramValues[ui.freq1->text()].append(msg->freq1);
	}
	if (paramsBitMask & BIT_FREQ2) {
		paramValues[ui.freq2->text()].append(msg->freq2);
	}
	if (paramsBitMask & BIT_F) {
		paramValues[ui.f->text()].append(msg->f);
	}
}

void BoardConsole::handleFreshMessage(const Message * msg) {
	if (firstMeasurement) {
		firstMeasurement = 0;
		startTime = QDateTime::currentMSecsSinceEpoch();
	}
	qint64 time = QDateTime::currentMSecsSinceEpoch() - startTime;

	if (plotCurveX.size() >= maxSize) {
		plotCurveX.pop_front();
	}
	plotCurveX.append(time);
	appendFreshData(msg);

	for (int i = 0; i < plots.size(); i++) {
		for (int j = 0; j < plotCurves[i].size(); j++) {
			QString paramName = plotLists[i]->item(j)->text();
			double paramValue = paramValues[paramName].last();
			if (plotCurveY[i][j].size() >= maxSize) {
				plotCurveY[i][j].pop_front();
			}
			plotCurveY[i][j].append(paramValue);

			int sizeDiff = plotCurveX.size() - plotCurveY[i][j].size();
			plotCurves[i][j]->setSamples(plotCurveX.mid(sizeDiff), plotCurveY[i][j]);
		}
		plots[i]->replot();
	}

	/*if (glwidget) {
		glwidget->setXRotation(msg->euler[0] * 16.0f);
		glwidget->setYRotation(msg->euler[1] * 16.0f);
		glwidget->setZRotation(msg->euler[2] * 16.0f);
	}*/
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

void BoardConsole::applyParamsMask() {
	if (stm) {
		stmReader->setParamsBitMask(paramsBitMask);
		QByteArray bitMaskCommand = number_command(PARAMS_BITMASK_SYMBOL, QString::number(paramsBitMask));
		stm->write(bitMaskCommand);
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
		}
		applyParamsMask();
	}
}

void BoardConsole::deleteListWidgetItems() {
	for (int i = 0; i < plotLists.size(); i++) {
		QListWidget *list = plotLists[i];
		if (list->hasFocus()) {
			foreach(QListWidgetItem *item, list->selectedItems()) {
				stopDisplayParam(i, item->text());
				delete item;
			}
		}
	}
}

void BoardConsole::mouseReleaseEvent(QMouseEvent *event) {
	if (event->button() != Qt::LeftButton) { return; }
	if (!draggedParam) { return; }
	QRect listParentRect = ui.visualGroupBox->geometry();
	QPoint cursor = event->pos();
	cursor -= listParentRect.topLeft();
	for (int i = 0; i < plotLists.size(); i++) {
		QListWidget *list = plotLists[i];
		QRect listRect = list->geometry();
		if (listRect.contains(cursor)) {
			QString newItemText = draggedParam->text();
			QList<QListWidgetItem *> sameItem = list->findItems(newItemText, Qt::MatchFixedString);
			if (sameItem.empty()) {
				list->addItem(newItemText);
				startDisplayParam(i, newItemText);
			}
			break;
		}
	}
	draggedParam = NULL;
}

void BoardConsole::stopDisplayParam(const QString &paramName) {
	for (int i = 0; i < plotLists.size(); i++) {
		QListWidget *list = plotLists[i];
		for (int j = 0; j < list->count(); j++) {
			QListWidgetItem *item = list->item(j);
			if (item->text() == paramName) {
				plotCurves[i][j]->detach();
				delete plotCurves[i][j];
				plotCurves[i].remove(j);
				plotCurveY[i].remove(j);
				delete item;
			}
		}
	}
}

void BoardConsole::stopDisplayParam(int plotIndex, const QString &paramName) {
	QListWidget *list = plotLists[plotIndex];
	for (int j = 0; j < list->count(); j++) {
		QListWidgetItem *item = list->item(j);
		if (item->text() == paramName) {
			plotCurves[plotIndex][j]->detach();
			delete plotCurves[plotIndex][j];
			plotCurves[plotIndex].remove(j);
			plotCurveY[plotIndex].remove(j);
		}
	}
}

void BoardConsole::startDisplayParam(int plotIndex, const QString &paramName) {
	QwtPlotCurve *newCurve = new QwtPlotCurve();
	newCurve->setPen(curveColors[plotCurves[plotIndex].size()]);
	plotCurves[plotIndex].append(newCurve);
	plotCurveY[plotIndex].append(QVector<QVector<double> >(1));
	newCurve->attach(plots[plotIndex]);
}

void BoardConsole::changeParamPlot(int plotFrom, int plotFromCurve, int plotTo, int plotToCurve) {
	QwtPlotCurve *curveToInsert = plotCurves[plotFrom][plotFromCurve];
	plotCurves[plotFrom].remove(plotFromCurve);
	plotCurves[plotTo].insert(plotToCurve, curveToInsert);
	curveToInsert->detach();
	curveToInsert->attach(plots[plotTo]);
	curveToInsert->setPen(curveColors[plotToCurve]);
	

	QVector<double> dataToInsert = plotCurveY[plotFrom][plotFromCurve];
	plotCurveY[plotTo].insert(plotToCurve, dataToInsert);
	plotCurveY[plotFrom].remove(plotFromCurve);
}

void BoardConsole::handlePlotListItemChanged(QListWidgetItem *item) {
	static int plotTo = 0, plotToCurve = 0;
	QListWidget *sender = (QListWidget *)QObject::sender();
	for (int i = 0; i < plotLists.size(); i++) {
		QListWidget *list = plotLists[i];
		if (sender == list) {
			if (item->text() == "") {
				for (int j = 0; j < list->count(); j++) {
					if (list->item(j) == item) {
						changeParamPlot(i, j, plotTo, plotToCurve);
						delete item;
						return;
					}
				}
				//stopDisplayParam("");
			} else {
				plotTo = i;
				for (int j = 0; j < list->count(); j++) {
					if (list->item(j) == item) {
						plotToCurve = j;
						return;
					}
				}
				//startDisplayParam(i, item->text());
			}
		}
	}
}
