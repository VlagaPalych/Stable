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
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qwt_plot.h"

QT_BEGIN_NAMESPACE

class Ui_BoardConsoleClass
{
public:
    QHBoxLayout *horizontalLayout_12;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_6;
    QGroupBox *connectGroupBox;
    QVBoxLayout *verticalLayout;
    QComboBox *serialComboBox;
    QPushButton *connectButton;
    QGroupBox *accelgroupBox;
    QVBoxLayout *verticalLayout_6;
    QRadioButton *HZ100RadioButton;
    QRadioButton *HZ800RadioButton;
    QGroupBox *filtergroupBox;
    QVBoxLayout *verticalLayout_4;
    QCheckBox *noFilterCheckBox;
    QHBoxLayout *horizontalLayout_4;
    QCheckBox *averagingAngleCheckBox;
    QSpinBox *angleWindowSpinBox;
    QHBoxLayout *horizontalLayout_5;
    QCheckBox *averagingAngVelCheckBox;
    QSpinBox *angVelWindowSpinBox;
    QCheckBox *kalmanFilterCheckBox;
    QGroupBox *stabGroupBox;
    QVBoxLayout *verticalLayout_2;
    QCheckBox *impulseCheckBox;
    QPushButton *stabToggleButton;
    QPushButton *calibrButton;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label;
    QSpinBox *pwm1SpinBox;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_2;
    QSpinBox *pwm2SpinBox;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_5;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pButton;
    QLineEdit *pLineEdit;
    QHBoxLayout *horizontalLayout_9;
    QPushButton *iButton;
    QLineEdit *iLineEdit;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *dButton;
    QLineEdit *dLineEdit;
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
        BoardConsoleClass->resize(713, 975);
        horizontalLayout_12 = new QHBoxLayout(BoardConsoleClass);
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        connectGroupBox = new QGroupBox(BoardConsoleClass);
        connectGroupBox->setObjectName(QStringLiteral("connectGroupBox"));
        verticalLayout = new QVBoxLayout(connectGroupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        serialComboBox = new QComboBox(connectGroupBox);
        serialComboBox->setObjectName(QStringLiteral("serialComboBox"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(serialComboBox->sizePolicy().hasHeightForWidth());
        serialComboBox->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(serialComboBox);

        connectButton = new QPushButton(connectGroupBox);
        connectButton->setObjectName(QStringLiteral("connectButton"));
        sizePolicy.setHeightForWidth(connectButton->sizePolicy().hasHeightForWidth());
        connectButton->setSizePolicy(sizePolicy);

        verticalLayout->addWidget(connectButton);


        horizontalLayout_6->addWidget(connectGroupBox);

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


        horizontalLayout_6->addWidget(accelgroupBox);


        verticalLayout_8->addLayout(horizontalLayout_6);

        filtergroupBox = new QGroupBox(BoardConsoleClass);
        filtergroupBox->setObjectName(QStringLiteral("filtergroupBox"));
        verticalLayout_4 = new QVBoxLayout(filtergroupBox);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        noFilterCheckBox = new QCheckBox(filtergroupBox);
        noFilterCheckBox->setObjectName(QStringLiteral("noFilterCheckBox"));

        verticalLayout_4->addWidget(noFilterCheckBox);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        averagingAngleCheckBox = new QCheckBox(filtergroupBox);
        averagingAngleCheckBox->setObjectName(QStringLiteral("averagingAngleCheckBox"));

        horizontalLayout_4->addWidget(averagingAngleCheckBox);

        angleWindowSpinBox = new QSpinBox(filtergroupBox);
        angleWindowSpinBox->setObjectName(QStringLiteral("angleWindowSpinBox"));
        angleWindowSpinBox->setMinimum(1);
        angleWindowSpinBox->setMaximum(24);
        angleWindowSpinBox->setValue(8);

        horizontalLayout_4->addWidget(angleWindowSpinBox);


        verticalLayout_4->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        averagingAngVelCheckBox = new QCheckBox(filtergroupBox);
        averagingAngVelCheckBox->setObjectName(QStringLiteral("averagingAngVelCheckBox"));

        horizontalLayout_5->addWidget(averagingAngVelCheckBox);

        angVelWindowSpinBox = new QSpinBox(filtergroupBox);
        angVelWindowSpinBox->setObjectName(QStringLiteral("angVelWindowSpinBox"));
        angVelWindowSpinBox->setMinimum(1);
        angVelWindowSpinBox->setMaximum(24);
        angVelWindowSpinBox->setValue(8);

        horizontalLayout_5->addWidget(angVelWindowSpinBox);


        verticalLayout_4->addLayout(horizontalLayout_5);

        kalmanFilterCheckBox = new QCheckBox(filtergroupBox);
        kalmanFilterCheckBox->setObjectName(QStringLiteral("kalmanFilterCheckBox"));

        verticalLayout_4->addWidget(kalmanFilterCheckBox);


        verticalLayout_8->addWidget(filtergroupBox);

        stabGroupBox = new QGroupBox(BoardConsoleClass);
        stabGroupBox->setObjectName(QStringLiteral("stabGroupBox"));
        verticalLayout_2 = new QVBoxLayout(stabGroupBox);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        impulseCheckBox = new QCheckBox(stabGroupBox);
        impulseCheckBox->setObjectName(QStringLiteral("impulseCheckBox"));

        verticalLayout_2->addWidget(impulseCheckBox);

        stabToggleButton = new QPushButton(stabGroupBox);
        stabToggleButton->setObjectName(QStringLiteral("stabToggleButton"));
        sizePolicy.setHeightForWidth(stabToggleButton->sizePolicy().hasHeightForWidth());
        stabToggleButton->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(stabToggleButton);

        calibrButton = new QPushButton(stabGroupBox);
        calibrButton->setObjectName(QStringLiteral("calibrButton"));
        sizePolicy.setHeightForWidth(calibrButton->sizePolicy().hasHeightForWidth());
        calibrButton->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(calibrButton);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        label = new QLabel(stabGroupBox);
        label->setObjectName(QStringLiteral("label"));
        label->setAlignment(Qt::AlignCenter);

        horizontalLayout_10->addWidget(label);

        pwm1SpinBox = new QSpinBox(stabGroupBox);
        pwm1SpinBox->setObjectName(QStringLiteral("pwm1SpinBox"));
        sizePolicy.setHeightForWidth(pwm1SpinBox->sizePolicy().hasHeightForWidth());
        pwm1SpinBox->setSizePolicy(sizePolicy);
        pwm1SpinBox->setMinimum(1000);
        pwm1SpinBox->setMaximum(2000);

        horizontalLayout_10->addWidget(pwm1SpinBox);


        verticalLayout_2->addLayout(horizontalLayout_10);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        label_2 = new QLabel(stabGroupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setAlignment(Qt::AlignCenter);

        horizontalLayout_11->addWidget(label_2);

        pwm2SpinBox = new QSpinBox(stabGroupBox);
        pwm2SpinBox->setObjectName(QStringLiteral("pwm2SpinBox"));
        sizePolicy.setHeightForWidth(pwm2SpinBox->sizePolicy().hasHeightForWidth());
        pwm2SpinBox->setSizePolicy(sizePolicy);
        pwm2SpinBox->setMinimum(1000);
        pwm2SpinBox->setMaximum(2000);

        horizontalLayout_11->addWidget(pwm2SpinBox);


        verticalLayout_2->addLayout(horizontalLayout_11);


        verticalLayout_8->addWidget(stabGroupBox);

        groupBox = new QGroupBox(BoardConsoleClass);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout_5 = new QVBoxLayout(groupBox);
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setContentsMargins(11, 11, 11, 11);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setSpacing(6);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        pButton = new QPushButton(groupBox);
        pButton->setObjectName(QStringLiteral("pButton"));
        sizePolicy.setHeightForWidth(pButton->sizePolicy().hasHeightForWidth());
        pButton->setSizePolicy(sizePolicy);

        horizontalLayout_7->addWidget(pButton);

        pLineEdit = new QLineEdit(groupBox);
        pLineEdit->setObjectName(QStringLiteral("pLineEdit"));
        sizePolicy.setHeightForWidth(pLineEdit->sizePolicy().hasHeightForWidth());
        pLineEdit->setSizePolicy(sizePolicy);

        horizontalLayout_7->addWidget(pLineEdit);


        verticalLayout_5->addLayout(horizontalLayout_7);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        iButton = new QPushButton(groupBox);
        iButton->setObjectName(QStringLiteral("iButton"));

        horizontalLayout_9->addWidget(iButton);

        iLineEdit = new QLineEdit(groupBox);
        iLineEdit->setObjectName(QStringLiteral("iLineEdit"));

        horizontalLayout_9->addWidget(iLineEdit);


        verticalLayout_5->addLayout(horizontalLayout_9);

        horizontalLayout_8 = new QHBoxLayout();
        horizontalLayout_8->setSpacing(6);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        dButton = new QPushButton(groupBox);
        dButton->setObjectName(QStringLiteral("dButton"));
        sizePolicy.setHeightForWidth(dButton->sizePolicy().hasHeightForWidth());
        dButton->setSizePolicy(sizePolicy);

        horizontalLayout_8->addWidget(dButton);

        dLineEdit = new QLineEdit(groupBox);
        dLineEdit->setObjectName(QStringLiteral("dLineEdit"));
        sizePolicy.setHeightForWidth(dLineEdit->sizePolicy().hasHeightForWidth());
        dLineEdit->setSizePolicy(sizePolicy);

        horizontalLayout_8->addWidget(dLineEdit);


        verticalLayout_5->addLayout(horizontalLayout_8);


        verticalLayout_8->addWidget(groupBox);


        horizontalLayout_12->addLayout(verticalLayout_8);

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
        saveToFileCheckBox->setChecked(false);

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
        pwm1CheckBox->setChecked(false);

        horizontalLayout->addWidget(pwm1CheckBox);

        pwm2CheckBox = new QCheckBox(telemetryGroupBox);
        pwm2CheckBox->setObjectName(QStringLiteral("pwm2CheckBox"));
        pwm2CheckBox->setChecked(false);

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


        horizontalLayout_12->addLayout(verticalLayout_7);


        retranslateUi(BoardConsoleClass);

        QMetaObject::connectSlotsByName(BoardConsoleClass);
    } // setupUi

    void retranslateUi(QWidget *BoardConsoleClass)
    {
        BoardConsoleClass->setWindowTitle(QApplication::translate("BoardConsoleClass", "BoardConsole", 0));
        connectGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "STM", 0));
        connectButton->setText(QApplication::translate("BoardConsoleClass", "Connect", 0));
        accelgroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Accelerometer", 0));
        HZ100RadioButton->setText(QApplication::translate("BoardConsoleClass", "100 Hz", 0));
        HZ800RadioButton->setText(QApplication::translate("BoardConsoleClass", "800 Hz", 0));
        filtergroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Filters", 0));
        noFilterCheckBox->setText(QApplication::translate("BoardConsoleClass", "No Filter", 0));
        averagingAngleCheckBox->setText(QApplication::translate("BoardConsoleClass", "AveragingAngle", 0));
        averagingAngVelCheckBox->setText(QApplication::translate("BoardConsoleClass", "AveraginVelo", 0));
        kalmanFilterCheckBox->setText(QApplication::translate("BoardConsoleClass", "Kalman", 0));
        stabGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Stabilization", 0));
        impulseCheckBox->setText(QApplication::translate("BoardConsoleClass", "impulse", 0));
        stabToggleButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
        calibrButton->setText(QApplication::translate("BoardConsoleClass", "Calibrate", 0));
        label->setText(QApplication::translate("BoardConsoleClass", "PWM1", 0));
        label_2->setText(QApplication::translate("BoardConsoleClass", "PWM2", 0));
        groupBox->setTitle(QApplication::translate("BoardConsoleClass", "PID Control", 0));
        pButton->setText(QApplication::translate("BoardConsoleClass", "P", 0));
        iButton->setText(QApplication::translate("BoardConsoleClass", "I", 0));
        dButton->setText(QApplication::translate("BoardConsoleClass", "D", 0));
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
