/********************************************************************************
** Form generated from reading UI file 'console.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONSOLE_H
#define UI_CONSOLE_H

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

class Ui_ConsoleClass
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
    QwtPlot *qwtPlot;

    void setupUi(QWidget *ConsoleClass)
    {
        if (ConsoleClass->objectName().isEmpty())
            ConsoleClass->setObjectName(QStringLiteral("ConsoleClass"));
        ConsoleClass->resize(377, 308);
        horizontalLayout_2 = new QHBoxLayout(ConsoleClass);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        connectGroupBox = new QGroupBox(ConsoleClass);
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

        stabGroupBox = new QGroupBox(ConsoleClass);
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
        telemetryGroupBox = new QGroupBox(ConsoleClass);
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


        verticalLayout_4->addWidget(telemetryGroupBox);

        qwtPlot = new QwtPlot(ConsoleClass);
        qwtPlot->setObjectName(QStringLiteral("qwtPlot"));

        verticalLayout_4->addWidget(qwtPlot);


        horizontalLayout_2->addLayout(verticalLayout_4);


        retranslateUi(ConsoleClass);

        QMetaObject::connectSlotsByName(ConsoleClass);
    } // setupUi

    void retranslateUi(QWidget *ConsoleClass)
    {
        ConsoleClass->setWindowTitle(QApplication::translate("ConsoleClass", "Console", 0));
        connectGroupBox->setTitle(QApplication::translate("ConsoleClass", "STM", 0));
        connectButton->setText(QApplication::translate("ConsoleClass", "Connect", 0));
        stabGroupBox->setTitle(QApplication::translate("ConsoleClass", "Stabilization", 0));
        stabOnButton->setText(QApplication::translate("ConsoleClass", "Turn On", 0));
        stabOffButton->setText(QApplication::translate("ConsoleClass", "Turn Off", 0));
        calibrButton->setText(QApplication::translate("ConsoleClass", "Calibrate", 0));
        telemetryGroupBox->setTitle(QApplication::translate("ConsoleClass", "Telemetry", 0));
        fullRadioButton->setText(QApplication::translate("ConsoleClass", "Full", 0));
        accelRadioButton->setText(QApplication::translate("ConsoleClass", "Acceleration", 0));
    } // retranslateUi

};

namespace Ui {
    class ConsoleClass: public Ui_ConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONSOLE_H
