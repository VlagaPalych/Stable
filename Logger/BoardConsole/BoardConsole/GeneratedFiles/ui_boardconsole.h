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
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qwt_plot.h"

QT_BEGIN_NAMESPACE

class Ui_BoardConsoleClass
{
public:
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_4;
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
    QVBoxLayout *verticalLayout_6;
    QRadioButton *HZ100RadioButton;
    QRadioButton *HZ800RadioButton;
    QGroupBox *stabGroupBox;
    QPushButton *stabToggleButton;
    QPushButton *calibrButton;
    QPushButton *k1Button;
    QLineEdit *k1LineEdit;
    QPushButton *k2Button;
    QLineEdit *k2LineEdit;
    QVBoxLayout *verticalLayout_7;
    QGroupBox *telemetryGroupBox;
    QHBoxLayout *horizontalLayout_3;
    QCheckBox *saveToFileCheckBox;
    QVBoxLayout *verticalLayout_3;
    QHBoxLayout *horizontalLayout_2;
    QRadioButton *fullRadioButton;
    QRadioButton *moveRadioButton;
    QRadioButton *axRadioButton;
    QRadioButton *ayRadioButton;
    QRadioButton *azRadioButton;
    QHBoxLayout *horizontalLayout;
    QCheckBox *angleCheckBox;
    QCheckBox *angVelCheckBox;
    QCheckBox *fCheckBox;
    QCheckBox *pwm1CheckBox;
    QCheckBox *pwm2CheckBox;
    QPushButton *telemetryToggleButton;
    QwtPlot *plot1;
    QwtPlot *plot2;

    void setupUi(QWidget *BoardConsoleClass)
    {
        if (BoardConsoleClass->objectName().isEmpty())
            BoardConsoleClass->setObjectName(QStringLiteral("BoardConsoleClass"));
        BoardConsoleClass->resize(630, 575);
        horizontalLayout_4 = new QHBoxLayout(BoardConsoleClass);
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
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


        verticalLayout_4->addWidget(connectGroupBox);

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


        verticalLayout_4->addWidget(filtergroupBox);

        accelgroupBox = new QGroupBox(BoardConsoleClass);
        accelgroupBox->setObjectName(QStringLiteral("accelgroupBox"));
        verticalLayout_6 = new QVBoxLayout(accelgroupBox);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        HZ100RadioButton = new QRadioButton(accelgroupBox);
        HZ100RadioButton->setObjectName(QStringLiteral("HZ100RadioButton"));
        HZ100RadioButton->setChecked(true);

        verticalLayout_6->addWidget(HZ100RadioButton);

        HZ800RadioButton = new QRadioButton(accelgroupBox);
        HZ800RadioButton->setObjectName(QStringLiteral("HZ800RadioButton"));
        HZ800RadioButton->setChecked(false);

        verticalLayout_6->addWidget(HZ800RadioButton);


        verticalLayout_4->addWidget(accelgroupBox);

        stabGroupBox = new QGroupBox(BoardConsoleClass);
        stabGroupBox->setObjectName(QStringLiteral("stabGroupBox"));
        stabToggleButton = new QPushButton(stabGroupBox);
        stabToggleButton->setObjectName(QStringLiteral("stabToggleButton"));
        stabToggleButton->setGeometry(QRect(10, 23, 75, 23));
        calibrButton = new QPushButton(stabGroupBox);
        calibrButton->setObjectName(QStringLiteral("calibrButton"));
        calibrButton->setGeometry(QRect(10, 52, 75, 23));
        k1Button = new QPushButton(stabGroupBox);
        k1Button->setObjectName(QStringLiteral("k1Button"));
        k1Button->setGeometry(QRect(10, 81, 75, 23));
        k1LineEdit = new QLineEdit(stabGroupBox);
        k1LineEdit->setObjectName(QStringLiteral("k1LineEdit"));
        k1LineEdit->setGeometry(QRect(10, 110, 133, 20));
        k2Button = new QPushButton(stabGroupBox);
        k2Button->setObjectName(QStringLiteral("k2Button"));
        k2Button->setGeometry(QRect(10, 136, 75, 23));
        k2LineEdit = new QLineEdit(stabGroupBox);
        k2LineEdit->setObjectName(QStringLiteral("k2LineEdit"));
        k2LineEdit->setGeometry(QRect(10, 165, 133, 20));

        verticalLayout_4->addWidget(stabGroupBox);


        horizontalLayout_4->addLayout(verticalLayout_4);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        telemetryGroupBox = new QGroupBox(BoardConsoleClass);
        telemetryGroupBox->setObjectName(QStringLiteral("telemetryGroupBox"));
        horizontalLayout_3 = new QHBoxLayout(telemetryGroupBox);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        saveToFileCheckBox = new QCheckBox(telemetryGroupBox);
        saveToFileCheckBox->setObjectName(QStringLiteral("saveToFileCheckBox"));

        horizontalLayout_3->addWidget(saveToFileCheckBox);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        fullRadioButton = new QRadioButton(telemetryGroupBox);
        fullRadioButton->setObjectName(QStringLiteral("fullRadioButton"));
        fullRadioButton->setChecked(true);

        horizontalLayout_2->addWidget(fullRadioButton);

        moveRadioButton = new QRadioButton(telemetryGroupBox);
        moveRadioButton->setObjectName(QStringLiteral("moveRadioButton"));

        horizontalLayout_2->addWidget(moveRadioButton);

        axRadioButton = new QRadioButton(telemetryGroupBox);
        axRadioButton->setObjectName(QStringLiteral("axRadioButton"));
        axRadioButton->setChecked(false);

        horizontalLayout_2->addWidget(axRadioButton);

        ayRadioButton = new QRadioButton(telemetryGroupBox);
        ayRadioButton->setObjectName(QStringLiteral("ayRadioButton"));

        horizontalLayout_2->addWidget(ayRadioButton);

        azRadioButton = new QRadioButton(telemetryGroupBox);
        azRadioButton->setObjectName(QStringLiteral("azRadioButton"));
        azRadioButton->setChecked(false);

        horizontalLayout_2->addWidget(azRadioButton);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        angleCheckBox = new QCheckBox(telemetryGroupBox);
        angleCheckBox->setObjectName(QStringLiteral("angleCheckBox"));
        angleCheckBox->setChecked(true);

        horizontalLayout->addWidget(angleCheckBox);

        angVelCheckBox = new QCheckBox(telemetryGroupBox);
        angVelCheckBox->setObjectName(QStringLiteral("angVelCheckBox"));
        angVelCheckBox->setChecked(true);

        horizontalLayout->addWidget(angVelCheckBox);

        fCheckBox = new QCheckBox(telemetryGroupBox);
        fCheckBox->setObjectName(QStringLiteral("fCheckBox"));
        fCheckBox->setChecked(true);

        horizontalLayout->addWidget(fCheckBox);

        pwm1CheckBox = new QCheckBox(telemetryGroupBox);
        pwm1CheckBox->setObjectName(QStringLiteral("pwm1CheckBox"));
        pwm1CheckBox->setChecked(true);

        horizontalLayout->addWidget(pwm1CheckBox);

        pwm2CheckBox = new QCheckBox(telemetryGroupBox);
        pwm2CheckBox->setObjectName(QStringLiteral("pwm2CheckBox"));
        pwm2CheckBox->setChecked(true);

        horizontalLayout->addWidget(pwm2CheckBox);


        verticalLayout_3->addLayout(horizontalLayout);


        horizontalLayout_3->addLayout(verticalLayout_3);

        telemetryToggleButton = new QPushButton(telemetryGroupBox);
        telemetryToggleButton->setObjectName(QStringLiteral("telemetryToggleButton"));

        horizontalLayout_3->addWidget(telemetryToggleButton);


        verticalLayout_7->addWidget(telemetryGroupBox);

        plot1 = new QwtPlot(BoardConsoleClass);
        plot1->setObjectName(QStringLiteral("plot1"));

        verticalLayout_7->addWidget(plot1);

        plot2 = new QwtPlot(BoardConsoleClass);
        plot2->setObjectName(QStringLiteral("plot2"));

        verticalLayout_7->addWidget(plot2);


        horizontalLayout_4->addLayout(verticalLayout_7);


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
        k1Button->setText(QApplication::translate("BoardConsoleClass", "K1", 0));
        k2Button->setText(QApplication::translate("BoardConsoleClass", "K2", 0));
        telemetryGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Telemetry", 0));
        saveToFileCheckBox->setText(QApplication::translate("BoardConsoleClass", "Save to file", 0));
        fullRadioButton->setText(QApplication::translate("BoardConsoleClass", "Full", 0));
        moveRadioButton->setText(QApplication::translate("BoardConsoleClass", "Move", 0));
        axRadioButton->setText(QApplication::translate("BoardConsoleClass", "AX", 0));
        ayRadioButton->setText(QApplication::translate("BoardConsoleClass", "AY", 0));
        azRadioButton->setText(QApplication::translate("BoardConsoleClass", "AZ", 0));
        angleCheckBox->setText(QApplication::translate("BoardConsoleClass", "Angle", 0));
        angVelCheckBox->setText(QApplication::translate("BoardConsoleClass", "AngVel", 0));
        fCheckBox->setText(QApplication::translate("BoardConsoleClass", "F", 0));
        pwm1CheckBox->setText(QApplication::translate("BoardConsoleClass", "pwm1", 0));
        pwm2CheckBox->setText(QApplication::translate("BoardConsoleClass", "pwm2", 0));
        telemetryToggleButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
    } // retranslateUi

};

namespace Ui {
    class BoardConsoleClass: public Ui_BoardConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOARDCONSOLE_H
