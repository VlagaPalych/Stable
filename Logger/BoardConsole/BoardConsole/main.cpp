#include "boardconsole.h"
#include <QtWidgets/QApplication>

int main(int argc, char *argv[])
{
	QApplication a(argc, argv);
	BoardConsole w;
	w.show();
	return a.exec();
}
