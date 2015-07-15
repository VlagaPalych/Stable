/********************************************************************************
** Form generated from reading UI file 'boardconsole.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BOARDCONSOLE_H
#define UI_BOARDCONSOLE_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qwt_plot.h"

QT_BEGIN_NAMESPACE

class Ui_BoardConsoleClass
{
public:
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_3;
    QGroupBox *connectGroupBox;
    QVBoxLayout *verticalLayout;
    QComboBox *serialComboBox;
    QPushButton *connectButton;
    QGroupBox *stabGroupBox;
    QVBoxLayout *verticalLayout_2;
    QPushButton *stabOnButton;
    QPushButton *stabOffButton;
    QPushButton *calibrButton;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *telemetryGroupBox;
    QHBoxLayout *horizontalLayout;
    QRadioButton *fullRadioButton;
    QRadioButton *accelRadioButton;
    QVBoxLayout *verticalLayout_5;
    QPushButton *telemetryStartButton;
    QPushButton *telemetryStopButton;
    QwtPlot *qwtPlot;

    void setupUi(QWidget *BoardConsoleClass)
    {
        if (BoardConsoleClass->objectName().isEmpty())
            BoardConsoleClass->setObjectName(QStringLiteral("BoardConsoleClass"));
        BoardConsoleClass->resize(517, 430);
        horizontalLayout_2 = new QHBoxLayout(BoardConsoleClass);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        connectGroupBox = new QGroupBox(BoardConsoleClass);
        connectGroupBox->setObjectName(QStringLiteral("connectGroupBox"));
        verticalLayout = new QVBoxLayout(connectGroupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        serialComboBox = new QComboBox(connectGroupBox);
        serialComboBox->setObjectName(QStringLiteral("serialComboBox"));

        verticalLayout->addWidget(serialComboBox);

        connectButton = new QPushButton(connectGroupBox);
        connectButton->setObjectName(QStringLiteral("connectButton"));

        verticalLayout->addWidget(connectButton);


        verticalLayout_3->addWidget(connectGroupBox);

        stabGroupBox = new QGroupBox(BoardConsoleClass);
        stabGroupBox->setObjectName(QStringLiteral("stabGroupBox"));
        verticalLayout_2 = new QVBoxLayout(stabGroupBox);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        stabOnButton = new QPushButton(stabGroupBox);
        stabOnButton->setObjectName(QStringLiteral("stabOnButton"));

        verticalLayout_2->addWidget(stabOnButton);

        stabOffButton = new QPushButton(stabGroupBox);
        stabOffButton->setObjectName(QStringLiteral("stabOffButton"));

        verticalLayout_2->addWidget(stabOffButton);

        calibrButton = new QPushButton(stabGroupBox);
        calibrButton->setObjectName(QStringLiteral("calibrButton"));

        verticalLayout_2->addWidget(calibrButton);


        verticalLayout_3->addWidget(stabGroupBox);


        horizontalLayout_2->addLayout(verticalLayout_3);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        telemetryGroupBox = new QGroupBox(BoardConsoleClass);
        telemetryGroupBox->setObjectName(QStringLiteral("telemetryGroupBox"));
        horizontalLayout = new QHBoxLayout(telemetryGroupBox);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        fullRadioButton = new QRadioButton(telemetryGroupBox);
        fullRadioButton->setObjectName(QStringLiteral("fullRadioButton"));

        horizontalLayout->addWidget(fullRadioButton);

        accelRadioButton = new QRadioButton(telemetryGroupBox);
        accelRadioButton->setObjectName(QStringLiteral("accelRadioButton"));

        horizontalLayout->addWidget(accelRadioButton);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        telemetryStartButton = new QPushButton(telemetryGroupBox);
        telemetryStartButton->setObjectName(QStringLiteral("telemetryStartButton"));

        verticalLayout_5->addWidget(telemetryStartButton);

        telemetryStopButton = new QPushButton(telemetryGroupBox);
        telemetryStopButton->setObjectName(QStringLiteral("telemetryStopButton"));

        verticalLayout_5->addWidget(telemetryStopButton);


        horizontalLayout->addLayout(verticalLayout_5);


        verticalLayout_4->addWidget(telemetryGroupBox);

        qwtPlot = new QwtPlot(BoardConsoleClass);
        qwtPlot->setObjectName(QStringLiteral("qwtPlot"));

        verticalLayout_4->addWidget(qwtPlot);


        horizontalLayout_2->addLayout(verticalLayout_4);


        retranslateUi(BoardConsoleClass);

        QMetaObject::connectSlotsByName(BoardConsoleClass);
    } // setupUi

    void retranslateUi(QWidget *BoardConsoleClass)
    {
        BoardConsoleClass->setWindowTitle(QApplication::translate("BoardConsoleClass", "BoardConsole", 0));
        connectGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "STM", 0));
        connectButton->setText(QApplication::translate("BoardConsoleClass", "Connect", 0));
        stabGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Stabilization", 0));
        stabOnButton->setText(QApplication::translate("BoardConsoleClass", "Turn On", 0));
        stabOffButton->setText(QApplication::translate("BoardConsoleClass", "Turn Off", 0));
        calibrButton->setText(QApplication::translate("BoardConsoleClass", "Calibrate", 0));
        telemetryGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Telemetry", 0));
        fullRadioButton->setText(QApplication::translate("BoardConsoleClass", "Full", 0));
        accelRadioButton->setText(QApplication::translate("BoardConsoleClass", "Acceleration", 0));
        telemetryStartButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
        telemetryStopButton->setText(QApplication::translate("BoardConsoleClass", "Stop", 0));
    } // retranslateUi

};

namespace Ui {
    class BoardConsoleClass: public Ui_BoardConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOARDCONSOLE_H
