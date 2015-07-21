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

	QString defineLogFile();

	bool firstMeasurement;
	int maxSize;
	qint64 startTime;

private Q_SLOTS:
	void handleConnectButton();
	void handleStabToggleButton();
	void handleCalibrButton();
	void handleTelemetryButtons();
	void handleTelemetryToggleButton();
	void handleAccelButtons();

	void handleFreshLine(QString &line);

	void handleNoFilterCheckBox();
	void handleKalmanFilterCheckBox();
	void handleAveragingCheckBox();

	void handleSaveToFileCheckBox();

	void handleK1Button();
	void handleK2Button();

	void handleTelemetryDisplayButtons();
};

#endif // BOARDCONSOLE_H
