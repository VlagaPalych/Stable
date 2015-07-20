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
	QVector<QVector<double> > yData;
	QVector<double> xData;

	QString defineLogFile();

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
};

#endif // BOARDCONSOLE_H
