#ifndef BOARDCONSOLE_H
#define BOARDCONSOLE_H

#include <QtWidgets/QWidget>
#include "ui_boardconsole.h"
#include <QtSerialPort\qserialport.h>
#include "serialportreader.h"
#include <qwt_plot_curve.h>
#include <qtimer.h>

class BoardConsole : public QWidget
{
	Q_OBJECT

public:
	BoardConsole(QWidget *parent = 0);
	~BoardConsole();

private:
	Ui::BoardConsoleClass ui;

	QSerialPort *stm;
	SerialPortReader *stmReader;

	QVector<QwtPlotCurve *> plot1_curves;
	QVector<QwtPlotCurve *> plot2_curves;

	QVector<double> angleX;
	QVector<double> angleY;

	QVector<double> angVelX;
	QVector<double> angVelY;

	QVector<double> fX;
	QVector<double> fY;

	QVector<double> pwm1X;
	QVector<double> pwm1Y;

	QVector<double> pwm2X;
	QVector<double> pwm2Y;

	QVector<double> gyroXX;
	QVector<double> gyroXY;

	QVector<double> gyroYX;
	QVector<double> gyroYY;

	QVector<double> gyroZX;
	QVector<double> gyroZY;

	QString defineLogFile();

	bool firstMeasurement;
	int maxSize;
	qint64 startTime;

	void STM_Init();
	QByteArray command(const char c);
	QByteArray number_command(const char c, QString number);

private Q_SLOTS:
void handleProgramButton();
	void handleConnectButton();
	void handleStabToggleButton();
	void handleCalibrButton();
	void handleClearTelemetryButton();
	void handleTelemetryToggleButton();
	void handleAccelButtons();
	void handleGyroButtons();

	void handleFreshLine(QString &line);

	void handleNoFilterCheckBox();
	void handleLowpassFilterCheckBox();
	void handleKalmanFilterCheckBox();
	void handleAveragingAngleCheckBox();
	void handleAveragingAngVelCheckBox();

	void handleAngleWindowSpinBox(int);
	void handleAngVelWindowSpinBox(int);

	void handleSaveToFileCheckBox();

	void handlePButton();
	void handleDButton();
	void handleIButton();

	void handleTelemetryDisplayButtons();

	void handlePwm1SpinBox(int);
	void handlePwm2SpinBox(int);
};

#endif // BOARDCONSOLE_H
