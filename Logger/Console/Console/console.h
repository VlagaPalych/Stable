#ifndef CONSOLE_H
#define CONSOLE_H

#include <QtWidgets/QWidget>
#include "ui_console.h"

class Console : public QWidget
{
	Q_OBJECT

public:
	Console(QWidget *parent = 0);
	~Console();

private:
	Ui::ConsoleClass ui;
};

#endif // CONSOLE_H
