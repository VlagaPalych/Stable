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
#include <QtWidgets/QCheckBox>
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
    QGroupBox *filtergroupBox;
    QVBoxLayout *verticalLayout_5;
    QCheckBox *noFilterCheckBox;
    QCheckBox *kalmanFilterCheckBox;
    QCheckBox *averagingCheckBox;
    QGroupBox *accelgroupBox;
    QRadioButton *HZ100RadioButton;
    QRadioButton *HZ800RadioButton;
    QGroupBox *stabGroupBox;
    QVBoxLayout *verticalLayout_2;
    QPushButton *stabToggleButton;
    QPushButton *calibrButton;
    QVBoxLayout *verticalLayout_4;
    QGroupBox *telemetryGroupBox;
    QHBoxLayout *horizontalLayout;
    QRadioButton *fullRadioButton;
    QRadioButton *axRadioButton;
    QRadioButton *ayRadioButton;
    QRadioButton *azRadioButton;
    QPushButton *telemetryToggleButton;
    QwtPlot *qwtPlot;

    void setupUi(QWidget *BoardConsoleClass)
    {
        if (BoardConsoleClass->objectName().isEmpty())
            BoardConsoleClass->setObjectName(QStringLiteral("BoardConsoleClass"));
        BoardConsoleClass->resize(809, 514);
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

        filtergroupBox = new QGroupBox(BoardConsoleClass);
        filtergroupBox->setObjectName(QStringLiteral("filtergroupBox"));
        verticalLayout_5 = new QVBoxLayout(filtergroupBox);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        noFilterCheckBox = new QCheckBox(filtergroupBox);
        noFilterCheckBox->setObjectName(QStringLiteral("noFilterCheckBox"));

        verticalLayout_5->addWidget(noFilterCheckBox);

        kalmanFilterCheckBox = new QCheckBox(filtergroupBox);
        kalmanFilterCheckBox->setObjectName(QStringLiteral("kalmanFilterCheckBox"));

        verticalLayout_5->addWidget(kalmanFilterCheckBox);

        averagingCheckBox = new QCheckBox(filtergroupBox);
        averagingCheckBox->setObjectName(QStringLiteral("averagingCheckBox"));

        verticalLayout_5->addWidget(averagingCheckBox);


        verticalLayout_3->addWidget(filtergroupBox);

        accelgroupBox = new QGroupBox(BoardConsoleClass);
        accelgroupBox->setObjectName(QStringLiteral("accelgroupBox"));
        HZ100RadioButton = new QRadioButton(accelgroupBox);
        HZ100RadioButton->setObjectName(QStringLiteral("HZ100RadioButton"));
        HZ100RadioButton->setGeometry(QRect(10, 20, 82, 17));
        HZ100RadioButton->setChecked(false);
        HZ800RadioButton = new QRadioButton(accelgroupBox);
        HZ800RadioButton->setObjectName(QStringLiteral("HZ800RadioButton"));
        HZ800RadioButton->setGeometry(QRect(10, 40, 82, 17));
        HZ800RadioButton->setChecked(true);

        verticalLayout_3->addWidget(accelgroupBox);

        stabGroupBox = new QGroupBox(BoardConsoleClass);
        stabGroupBox->setObjectName(QStringLiteral("stabGroupBox"));
        verticalLayout_2 = new QVBoxLayout(stabGroupBox);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        stabToggleButton = new QPushButton(stabGroupBox);
        stabToggleButton->setObjectName(QStringLiteral("stabToggleButton"));

        verticalLayout_2->addWidget(stabToggleButton);

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

        axRadioButton = new QRadioButton(telemetryGroupBox);
        axRadioButton->setObjectName(QStringLiteral("axRadioButton"));
        axRadioButton->setChecked(false);

        horizontalLayout->addWidget(axRadioButton);

        ayRadioButton = new QRadioButton(telemetryGroupBox);
        ayRadioButton->setObjectName(QStringLiteral("ayRadioButton"));

        horizontalLayout->addWidget(ayRadioButton);

        azRadioButton = new QRadioButton(telemetryGroupBox);
        azRadioButton->setObjectName(QStringLiteral("azRadioButton"));
        azRadioButton->setChecked(true);

        horizontalLayout->addWidget(azRadioButton);

        telemetryToggleButton = new QPushButton(telemetryGroupBox);
        telemetryToggleButton->setObjectName(QStringLiteral("telemetryToggleButton"));

        horizontalLayout->addWidget(telemetryToggleButton);


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
        filtergroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Filters", 0));
        noFilterCheckBox->setText(QApplication::translate("BoardConsoleClass", "No Filter", 0));
        kalmanFilterCheckBox->setText(QApplication::translate("BoardConsoleClass", "Kalman", 0));
        averagingCheckBox->setText(QApplication::translate("BoardConsoleClass", "Averaging", 0));
        accelgroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Accelerometer", 0));
        HZ100RadioButton->setText(QApplication::translate("BoardConsoleClass", "100 Hz", 0));
        HZ800RadioButton->setText(QApplication::translate("BoardConsoleClass", "800 Hz", 0));
        stabGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Stabilization", 0));
        stabToggleButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
        calibrButton->setText(QApplication::translate("BoardConsoleClass", "Calibrate", 0));
        telemetryGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Telemetry", 0));
        fullRadioButton->setText(QApplication::translate("BoardConsoleClass", "Full", 0));
        axRadioButton->setText(QApplication::translate("BoardConsoleClass", "AX", 0));
        ayRadioButton->setText(QApplication::translate("BoardConsoleClass", "AY", 0));
        azRadioButton->setText(QApplication::translate("BoardConsoleClass", "AZ", 0));
        telemetryToggleButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
    } // retranslateUi

};

namespace Ui {
    class BoardConsoleClass: public Ui_BoardConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOARDCONSOLE_H
