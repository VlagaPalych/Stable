#ifndef BOARDCONSOLE_H
#define BOARDCONSOLE_H

#include <QtWidgets/QWidget>
#include "ui_boardconsole.h"
#include <QtSerialPort\qserialport.h>
#include "serialportreader.h"
#include <qwt_plot_curve.h>
#include <qtimer.h>
#include "glwidget.h"
#include "qsettings.h"
#include "qshortcut.h"
#include "qmap.h"

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

	GLWidget *glwidget;
	QVector<QCheckBox *> paramCheckBoxes;
	QCheckBox *draggedParam;
	QVector<QListWidget *> plotLists;

	quint32 paramsBitMask;

	QSettings *settings;

	QVector<QwtPlot *> plots;
	QVector<QVector<QwtPlotCurve *> > plotCurves;
	QVector<double> plotCurveX; 
	QVector<QVector<QVector<double> > > plotCurveY;
	QMap<QString, QVector<double> > paramValues;
	QVector<QColor> curveColors;

	QString defineLogFile();

	bool firstMeasurement;
	int maxSize;
	qint64 startTime;

	void STM_Init();
	QByteArray command(const char c);
	QByteArray number_command(const char c, QString number);
	QByteArray double_number_command(const char c, QString num1, QString num2);

	void initPlots();
	void fillParamsVector();
	void fillListsVector();
	

	void stopDisplayParam(int plotIndex, const QString &paramName);
	void stopDisplayParam(const QString &paramName);
	void startDisplayParam(int plotIndex, const QString &paramName);
	void applyParamsMask();

	void readSettings();
	void writeSettings();

	void appendFreshData(const Message * msg);


private Q_SLOTS:
	void handleProgramButton();
	void handleConnectButton();
	void handleStopMotorsButton();
	void handleCalibrButton();
	void handleClearTelemetryButton();
	void handleTelemetryToggleButton();

	void handleSaveToFileCheckBox();

	void handlePButton();
	void handleDButton();
	void handleIButton();

	void handleTelemetryDisplayButtons();

	void handlePwm1SpinBox(int);
	void handlePwm2SpinBox(int);

	void handlePwm1Slider(int);
	void handlePwm2Slider(int);

	void handleResearchButtons();
	void handleFreshMessage(const Message *);

	void handleRotationButton();
	void paramPressed();
	void handleParamBoxStateChanged(int state);
	void mouseReleaseEvent(QMouseEvent * event);
	void handlePlotListItemChanged(QListWidgetItem *item);
	void deleteListWidgetItems();
};

#endif // BOARDCONSOLE_H
