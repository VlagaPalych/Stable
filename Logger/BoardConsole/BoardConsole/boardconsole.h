#ifndef BOARDCONSOLE_H
#define BOARDCONSOLE_H

#include <QtWidgets/QWidget>
#include "ui_boardconsole.h"
#include <QtSerialPort\qserialport.h>
#include "serialportreader.h"
#include <qwt_plot_curve.h>

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

	QVector<QwtPlotCurve *> acceleration;
	QVector<QVector<double> > accelData;
	QVector<double> xData;

	void defineLogFile();

private Q_SLOTS:
	void handleConnectButton();
	void handleStabOnButton();
	void handleStabOffButton();
	void handleCalibrButton();
	void handleTelemetryButtons();
	void handleTelemetryStartButton();
	void handleTelemetryStopButton();

	void handleFreshLine(QString &line);
};

#endif // BOARDCONSOLE_H
