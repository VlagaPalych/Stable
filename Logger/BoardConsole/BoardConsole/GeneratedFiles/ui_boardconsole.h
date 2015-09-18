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
    QVBoxLayout *verticalLayout_12;
    QHBoxLayout *horizontalLayout_2;
    QGroupBox *connectGroupBox;
    QVBoxLayout *verticalLayout;
    QComboBox *serialComboBox;
    QPushButton *connectButton;
    QPushButton *programButton;
    QGroupBox *accelgroupBox;
    QVBoxLayout *verticalLayout_6;
    QRadioButton *HZ25RadioButton;
    QRadioButton *HZ50RadioButton;
    QRadioButton *HZ100RadioButton;
    QRadioButton *HZ800RadioButton;
    QRadioButton *HZ1600RadioButton;
    QRadioButton *HZ3200RadioButton;
    QGroupBox *groupBox_4;
    QVBoxLayout *verticalLayout_10;
    QRadioButton *gyroHZ100RadioButton;
    QRadioButton *gyroHZ250RadioButton;
    QRadioButton *gyroHZ500RadioButton;
    QRadioButton *gyroHZ1000RadioButton;
    QHBoxLayout *horizontalLayout_6;
    QGroupBox *researchGroupBox;
    QVBoxLayout *verticalLayout_4;
    QCheckBox *lowpassFilterCheckBox;
    QRadioButton *simpleRadioButton;
    QRadioButton *noResearchRadioButton;
    QRadioButton *impulseRadioButton;
    QRadioButton *stepRadioButton;
    QRadioButton *sineRadioButton;
    QRadioButton *expRadioButton;
    QLabel *label_3;
    QLineEdit *researchAmplLineEdit;
    QLabel *label_4;
    QLineEdit *researchFreqLineEdit;
    QGroupBox *stabGroupBox;
    QVBoxLayout *verticalLayout_2;
    QPushButton *stabToggleButton;
    QPushButton *calibrButton;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label;
    QSpinBox *pwm1SpinBox;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_2;
    QSpinBox *pwm2SpinBox;
    QGroupBox *groupBox;
    QVBoxLayout *verticalLayout_8;
    QHBoxLayout *horizontalLayout_7;
    QPushButton *pButton;
    QLineEdit *pLineEdit;
    QHBoxLayout *horizontalLayout_9;
    QPushButton *iButton;
    QLineEdit *iLineEdit;
    QHBoxLayout *horizontalLayout_8;
    QPushButton *dButton;
    QLineEdit *dLineEdit;
    QVBoxLayout *verticalLayout_9;
    QHBoxLayout *horizontalLayout_3;
    QGroupBox *telemetryGroupBox;
    QVBoxLayout *verticalLayout_11;
    QCheckBox *saveToFileCheckBox;
    QPushButton *telemetryToggleButton;
    QPushButton *clearTelemetryButton;
    QGroupBox *groupBox_2;
    QVBoxLayout *verticalLayout_7;
    QCheckBox *angleCheckBox;
    QCheckBox *angVelCheckBox;
    QGroupBox *groupBox_3;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_3;
    QRadioButton *fRadioButton;
    QCheckBox *fCheckBox;
    QCheckBox *pwm1CheckBox;
    QCheckBox *pwm2CheckBox;
    QVBoxLayout *verticalLayout_5;
    QRadioButton *gyroRadioButton;
    QCheckBox *gxCheckBox;
    QCheckBox *gyCheckBox;
    QCheckBox *gzCheckBox;
    QwtPlot *plot1;
    QwtPlot *plot2;

    void setupUi(QWidget *BoardConsoleClass)
    {
        if (BoardConsoleClass->objectName().isEmpty())
            BoardConsoleClass->setObjectName(QStringLiteral("BoardConsoleClass"));
        BoardConsoleClass->resize(774, 1019);
        horizontalLayout_12 = new QHBoxLayout(BoardConsoleClass);
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
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

        programButton = new QPushButton(connectGroupBox);
        programButton->setObjectName(QStringLiteral("programButton"));

        verticalLayout->addWidget(programButton);


        horizontalLayout_2->addWidget(connectGroupBox);

        accelgroupBox = new QGroupBox(BoardConsoleClass);
        accelgroupBox->setObjectName(QStringLiteral("accelgroupBox"));
        verticalLayout_6 = new QVBoxLayout(accelgroupBox);
        verticalLayout_6->setSpacing(6);
        verticalLayout_6->setContentsMargins(11, 11, 11, 11);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        HZ25RadioButton = new QRadioButton(accelgroupBox);
        HZ25RadioButton->setObjectName(QStringLiteral("HZ25RadioButton"));

        verticalLayout_6->addWidget(HZ25RadioButton);

        HZ50RadioButton = new QRadioButton(accelgroupBox);
        HZ50RadioButton->setObjectName(QStringLiteral("HZ50RadioButton"));

        verticalLayout_6->addWidget(HZ50RadioButton);

        HZ100RadioButton = new QRadioButton(accelgroupBox);
        HZ100RadioButton->setObjectName(QStringLiteral("HZ100RadioButton"));
        HZ100RadioButton->setChecked(false);

        verticalLayout_6->addWidget(HZ100RadioButton);

        HZ800RadioButton = new QRadioButton(accelgroupBox);
        HZ800RadioButton->setObjectName(QStringLiteral("HZ800RadioButton"));
        HZ800RadioButton->setChecked(false);

        verticalLayout_6->addWidget(HZ800RadioButton);

        HZ1600RadioButton = new QRadioButton(accelgroupBox);
        HZ1600RadioButton->setObjectName(QStringLiteral("HZ1600RadioButton"));
        HZ1600RadioButton->setChecked(true);

        verticalLayout_6->addWidget(HZ1600RadioButton);

        HZ3200RadioButton = new QRadioButton(accelgroupBox);
        HZ3200RadioButton->setObjectName(QStringLiteral("HZ3200RadioButton"));

        verticalLayout_6->addWidget(HZ3200RadioButton);


        horizontalLayout_2->addWidget(accelgroupBox);

        groupBox_4 = new QGroupBox(BoardConsoleClass);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        verticalLayout_10 = new QVBoxLayout(groupBox_4);
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setContentsMargins(11, 11, 11, 11);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        gyroHZ100RadioButton = new QRadioButton(groupBox_4);
        gyroHZ100RadioButton->setObjectName(QStringLiteral("gyroHZ100RadioButton"));
        gyroHZ100RadioButton->setChecked(false);

        verticalLayout_10->addWidget(gyroHZ100RadioButton);

        gyroHZ250RadioButton = new QRadioButton(groupBox_4);
        gyroHZ250RadioButton->setObjectName(QStringLiteral("gyroHZ250RadioButton"));

        verticalLayout_10->addWidget(gyroHZ250RadioButton);

        gyroHZ500RadioButton = new QRadioButton(groupBox_4);
        gyroHZ500RadioButton->setObjectName(QStringLiteral("gyroHZ500RadioButton"));
        gyroHZ500RadioButton->setChecked(false);

        verticalLayout_10->addWidget(gyroHZ500RadioButton);

        gyroHZ1000RadioButton = new QRadioButton(groupBox_4);
        gyroHZ1000RadioButton->setObjectName(QStringLiteral("gyroHZ1000RadioButton"));
        gyroHZ1000RadioButton->setChecked(true);

        verticalLayout_10->addWidget(gyroHZ1000RadioButton);


        horizontalLayout_2->addWidget(groupBox_4);


        verticalLayout_12->addLayout(horizontalLayout_2);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        researchGroupBox = new QGroupBox(BoardConsoleClass);
        researchGroupBox->setObjectName(QStringLiteral("researchGroupBox"));
        verticalLayout_4 = new QVBoxLayout(researchGroupBox);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        lowpassFilterCheckBox = new QCheckBox(researchGroupBox);
        lowpassFilterCheckBox->setObjectName(QStringLiteral("lowpassFilterCheckBox"));
        lowpassFilterCheckBox->setChecked(true);

        verticalLayout_4->addWidget(lowpassFilterCheckBox);

        simpleRadioButton = new QRadioButton(researchGroupBox);
        simpleRadioButton->setObjectName(QStringLiteral("simpleRadioButton"));

        verticalLayout_4->addWidget(simpleRadioButton);

        noResearchRadioButton = new QRadioButton(researchGroupBox);
        noResearchRadioButton->setObjectName(QStringLiteral("noResearchRadioButton"));
        noResearchRadioButton->setChecked(true);

        verticalLayout_4->addWidget(noResearchRadioButton);

        impulseRadioButton = new QRadioButton(researchGroupBox);
        impulseRadioButton->setObjectName(QStringLiteral("impulseRadioButton"));

        verticalLayout_4->addWidget(impulseRadioButton);

        stepRadioButton = new QRadioButton(researchGroupBox);
        stepRadioButton->setObjectName(QStringLiteral("stepRadioButton"));

        verticalLayout_4->addWidget(stepRadioButton);

        sineRadioButton = new QRadioButton(researchGroupBox);
        sineRadioButton->setObjectName(QStringLiteral("sineRadioButton"));

        verticalLayout_4->addWidget(sineRadioButton);

        expRadioButton = new QRadioButton(researchGroupBox);
        expRadioButton->setObjectName(QStringLiteral("expRadioButton"));

        verticalLayout_4->addWidget(expRadioButton);

        label_3 = new QLabel(researchGroupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(label_3);

        researchAmplLineEdit = new QLineEdit(researchGroupBox);
        researchAmplLineEdit->setObjectName(QStringLiteral("researchAmplLineEdit"));
        sizePolicy.setHeightForWidth(researchAmplLineEdit->sizePolicy().hasHeightForWidth());
        researchAmplLineEdit->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(researchAmplLineEdit);

        label_4 = new QLabel(researchGroupBox);
        label_4->setObjectName(QStringLiteral("label_4"));
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(label_4);

        researchFreqLineEdit = new QLineEdit(researchGroupBox);
        researchFreqLineEdit->setObjectName(QStringLiteral("researchFreqLineEdit"));
        sizePolicy.setHeightForWidth(researchFreqLineEdit->sizePolicy().hasHeightForWidth());
        researchFreqLineEdit->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(researchFreqLineEdit);


        horizontalLayout_6->addWidget(researchGroupBox);

        stabGroupBox = new QGroupBox(BoardConsoleClass);
        stabGroupBox->setObjectName(QStringLiteral("stabGroupBox"));
        verticalLayout_2 = new QVBoxLayout(stabGroupBox);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
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
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy1);
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
        sizePolicy1.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy1);
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


        horizontalLayout_6->addWidget(stabGroupBox);


        verticalLayout_12->addLayout(horizontalLayout_6);

        groupBox = new QGroupBox(BoardConsoleClass);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        verticalLayout_8 = new QVBoxLayout(groupBox);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
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


        verticalLayout_8->addLayout(horizontalLayout_7);

        horizontalLayout_9 = new QHBoxLayout();
        horizontalLayout_9->setSpacing(6);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        iButton = new QPushButton(groupBox);
        iButton->setObjectName(QStringLiteral("iButton"));
        sizePolicy.setHeightForWidth(iButton->sizePolicy().hasHeightForWidth());
        iButton->setSizePolicy(sizePolicy);

        horizontalLayout_9->addWidget(iButton);

        iLineEdit = new QLineEdit(groupBox);
        iLineEdit->setObjectName(QStringLiteral("iLineEdit"));
        sizePolicy.setHeightForWidth(iLineEdit->sizePolicy().hasHeightForWidth());
        iLineEdit->setSizePolicy(sizePolicy);

        horizontalLayout_9->addWidget(iLineEdit);


        verticalLayout_8->addLayout(horizontalLayout_9);

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


        verticalLayout_8->addLayout(horizontalLayout_8);


        verticalLayout_12->addWidget(groupBox);


        horizontalLayout_12->addLayout(verticalLayout_12);

        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        telemetryGroupBox = new QGroupBox(BoardConsoleClass);
        telemetryGroupBox->setObjectName(QStringLiteral("telemetryGroupBox"));
        verticalLayout_11 = new QVBoxLayout(telemetryGroupBox);
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setContentsMargins(11, 11, 11, 11);
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        saveToFileCheckBox = new QCheckBox(telemetryGroupBox);
        saveToFileCheckBox->setObjectName(QStringLiteral("saveToFileCheckBox"));
        saveToFileCheckBox->setChecked(false);

        verticalLayout_11->addWidget(saveToFileCheckBox);

        telemetryToggleButton = new QPushButton(telemetryGroupBox);
        telemetryToggleButton->setObjectName(QStringLiteral("telemetryToggleButton"));

        verticalLayout_11->addWidget(telemetryToggleButton);

        clearTelemetryButton = new QPushButton(telemetryGroupBox);
        clearTelemetryButton->setObjectName(QStringLiteral("clearTelemetryButton"));

        verticalLayout_11->addWidget(clearTelemetryButton);


        horizontalLayout_3->addWidget(telemetryGroupBox);

        groupBox_2 = new QGroupBox(BoardConsoleClass);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        verticalLayout_7 = new QVBoxLayout(groupBox_2);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        angleCheckBox = new QCheckBox(groupBox_2);
        angleCheckBox->setObjectName(QStringLiteral("angleCheckBox"));
        angleCheckBox->setChecked(true);

        verticalLayout_7->addWidget(angleCheckBox);

        angVelCheckBox = new QCheckBox(groupBox_2);
        angVelCheckBox->setObjectName(QStringLiteral("angVelCheckBox"));
        angVelCheckBox->setChecked(true);

        verticalLayout_7->addWidget(angVelCheckBox);


        horizontalLayout_3->addWidget(groupBox_2);

        groupBox_3 = new QGroupBox(BoardConsoleClass);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        horizontalLayout = new QHBoxLayout(groupBox_3);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        fRadioButton = new QRadioButton(groupBox_3);
        fRadioButton->setObjectName(QStringLiteral("fRadioButton"));
        fRadioButton->setChecked(true);

        verticalLayout_3->addWidget(fRadioButton);

        fCheckBox = new QCheckBox(groupBox_3);
        fCheckBox->setObjectName(QStringLiteral("fCheckBox"));
        fCheckBox->setChecked(true);

        verticalLayout_3->addWidget(fCheckBox);

        pwm1CheckBox = new QCheckBox(groupBox_3);
        pwm1CheckBox->setObjectName(QStringLiteral("pwm1CheckBox"));
        pwm1CheckBox->setChecked(false);

        verticalLayout_3->addWidget(pwm1CheckBox);

        pwm2CheckBox = new QCheckBox(groupBox_3);
        pwm2CheckBox->setObjectName(QStringLiteral("pwm2CheckBox"));
        pwm2CheckBox->setChecked(false);

        verticalLayout_3->addWidget(pwm2CheckBox);


        horizontalLayout->addLayout(verticalLayout_3);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setSpacing(6);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        gyroRadioButton = new QRadioButton(groupBox_3);
        gyroRadioButton->setObjectName(QStringLiteral("gyroRadioButton"));
        gyroRadioButton->setChecked(false);

        verticalLayout_5->addWidget(gyroRadioButton);

        gxCheckBox = new QCheckBox(groupBox_3);
        gxCheckBox->setObjectName(QStringLiteral("gxCheckBox"));
        gxCheckBox->setChecked(true);

        verticalLayout_5->addWidget(gxCheckBox);

        gyCheckBox = new QCheckBox(groupBox_3);
        gyCheckBox->setObjectName(QStringLiteral("gyCheckBox"));

        verticalLayout_5->addWidget(gyCheckBox);

        gzCheckBox = new QCheckBox(groupBox_3);
        gzCheckBox->setObjectName(QStringLiteral("gzCheckBox"));

        verticalLayout_5->addWidget(gzCheckBox);


        horizontalLayout->addLayout(verticalLayout_5);


        horizontalLayout_3->addWidget(groupBox_3);


        verticalLayout_9->addLayout(horizontalLayout_3);

        plot1 = new QwtPlot(BoardConsoleClass);
        plot1->setObjectName(QStringLiteral("plot1"));

        verticalLayout_9->addWidget(plot1);

        plot2 = new QwtPlot(BoardConsoleClass);
        plot2->setObjectName(QStringLiteral("plot2"));

        verticalLayout_9->addWidget(plot2);


        horizontalLayout_12->addLayout(verticalLayout_9);


        retranslateUi(BoardConsoleClass);

        QMetaObject::connectSlotsByName(BoardConsoleClass);
    } // setupUi

    void retranslateUi(QWidget *BoardConsoleClass)
    {
        BoardConsoleClass->setWindowTitle(QApplication::translate("BoardConsoleClass", "BoardConsole", 0));
        connectGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "STM", 0));
        connectButton->setText(QApplication::translate("BoardConsoleClass", "Connect", 0));
        programButton->setText(QApplication::translate("BoardConsoleClass", "Program", 0));
        accelgroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Accel", 0));
        HZ25RadioButton->setText(QApplication::translate("BoardConsoleClass", "25 Hz", 0));
        HZ50RadioButton->setText(QApplication::translate("BoardConsoleClass", "50 Hz", 0));
        HZ100RadioButton->setText(QApplication::translate("BoardConsoleClass", "100 Hz", 0));
        HZ800RadioButton->setText(QApplication::translate("BoardConsoleClass", "800 Hz", 0));
        HZ1600RadioButton->setText(QApplication::translate("BoardConsoleClass", "1600 Hz", 0));
        HZ3200RadioButton->setText(QApplication::translate("BoardConsoleClass", "3200 Hz", 0));
        groupBox_4->setTitle(QApplication::translate("BoardConsoleClass", "Gyro", 0));
        gyroHZ100RadioButton->setText(QApplication::translate("BoardConsoleClass", "100 Hz", 0));
        gyroHZ250RadioButton->setText(QApplication::translate("BoardConsoleClass", "250 Hz", 0));
        gyroHZ500RadioButton->setText(QApplication::translate("BoardConsoleClass", "500 Hz", 0));
        gyroHZ1000RadioButton->setText(QApplication::translate("BoardConsoleClass", "1000 Hz", 0));
        researchGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Research", 0));
        lowpassFilterCheckBox->setText(QApplication::translate("BoardConsoleClass", "lowpass", 0));
        simpleRadioButton->setText(QApplication::translate("BoardConsoleClass", "Simple", 0));
        noResearchRadioButton->setText(QApplication::translate("BoardConsoleClass", "No Research", 0));
        impulseRadioButton->setText(QApplication::translate("BoardConsoleClass", "Impulse", 0));
        stepRadioButton->setText(QApplication::translate("BoardConsoleClass", "Step", 0));
        sineRadioButton->setText(QApplication::translate("BoardConsoleClass", "Sine", 0));
        expRadioButton->setText(QApplication::translate("BoardConsoleClass", "Exp", 0));
        label_3->setText(QApplication::translate("BoardConsoleClass", "Amplitude", 0));
        researchAmplLineEdit->setText(QApplication::translate("BoardConsoleClass", "1.0", 0));
        label_4->setText(QApplication::translate("BoardConsoleClass", "Frequency", 0));
        researchFreqLineEdit->setText(QApplication::translate("BoardConsoleClass", "0.01", 0));
        stabGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Stabilization", 0));
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
        telemetryToggleButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
        clearTelemetryButton->setText(QApplication::translate("BoardConsoleClass", "Clear", 0));
        groupBox_2->setTitle(QApplication::translate("BoardConsoleClass", "Plot 1", 0));
        angleCheckBox->setText(QApplication::translate("BoardConsoleClass", "Angle", 0));
        angVelCheckBox->setText(QApplication::translate("BoardConsoleClass", "AngVel", 0));
        groupBox_3->setTitle(QApplication::translate("BoardConsoleClass", "Plot 2", 0));
        fRadioButton->setText(QApplication::translate("BoardConsoleClass", "F", 0));
        fCheckBox->setText(QApplication::translate("BoardConsoleClass", "F", 0));
        pwm1CheckBox->setText(QApplication::translate("BoardConsoleClass", "pwm1", 0));
        pwm2CheckBox->setText(QApplication::translate("BoardConsoleClass", "pwm2", 0));
        gyroRadioButton->setText(QApplication::translate("BoardConsoleClass", "Gyro", 0));
        gxCheckBox->setText(QApplication::translate("BoardConsoleClass", "X", 0));
        gyCheckBox->setText(QApplication::translate("BoardConsoleClass", "Y", 0));
        gzCheckBox->setText(QApplication::translate("BoardConsoleClass", "Z", 0));
    } // retranslateUi

};

namespace Ui {
    class BoardConsoleClass: public Ui_BoardConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOARDCONSOLE_H
