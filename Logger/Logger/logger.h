#ifndef LOGGER_H
#define LOGGER_H

#include <QWidget>

namespace Ui {
class Logger;
}
class QString;
class QSerialPort;
class SerialPortReader;

class Logger : public QWidget
{
    Q_OBJECT

public:
    explicit Logger(QWidget *parent = 0);
    ~Logger();

private slots:
    void controlButtonToggled();
    void handleSpinBox(int);
    void handleStopButton();
    void handleReloadButton();
    void handleConnectButton();

private:
    void defineLogFile();

    Ui::Logger *ui;

    QSerialPort *stm;
    SerialPortReader *stmReader;
};

#endif // LOGGER_H
