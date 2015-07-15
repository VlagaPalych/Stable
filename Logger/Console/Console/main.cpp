#include "console.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	Console w;
	w.show();
	return a.exec();
}
