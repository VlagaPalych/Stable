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
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qwt_plot.h"

QT_BEGIN_NAMESPACE

class Ui_BoardConsoleClass
{
public:
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_12;
    QHBoxLayout *horizontalLayout;
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
    QCheckBox *turnUselessCheckBox;
    QCheckBox *gyroRecalibrationCheckBox;
    QRadioButton *noResearchRadioButton;
    QRadioButton *operatorRadioButton;
    QRadioButton *adjustRadioButton;
    QRadioButton *simpleRadioButton;
    QRadioButton *pidRadioButton;
    QRadioButton *impulseRadioButton;
    QRadioButton *stepRadioButton;
    QRadioButton *sineRadioButton;
    QRadioButton *expRadioButton;
    QLabel *label_3;
    QLineEdit *researchAmplLineEdit;
    QLabel *label_4;
    QLineEdit *researchFreqLineEdit;
    QPushButton *tranquilityButton;
    QLineEdit *tranquilityLineEdit;
    QPushButton *pwmStepButton;
    QLineEdit *pwmStepLineEdit;
    QPushButton *everyNButton;
    QLineEdit *everyNLineEdit;
    QGroupBox *stabGroupBox;
    QVBoxLayout *verticalLayout_2;
    QPushButton *stopMotorsButton;
    QPushButton *calibrButton;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label;
    QSpinBox *pwm1SpinBox;
    QHBoxLayout *horizontalLayout_11;
    QLabel *label_2;
    QSpinBox *pwm2SpinBox;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_5;
    QLabel *pwm1Label;
    QSlider *pwm1Slider;
    QHBoxLayout *horizontalLayout_10;
    QLabel *label_6;
    QLabel *pwm2Label;
    QSlider *pwm2Slider;
    QPushButton *maxAngleButton;
    QLineEdit *maxAngleLineEdit;
    QPushButton *maxVelButton;
    QLineEdit *maxVelLineEdit;
    QPushButton *turnoffAngleButton;
    QLineEdit *turnoffAngleLineEdit;
    QPushButton *accelDeviationButton;
    QLineEdit *accelDeviationLineEdit;
    QHBoxLayout *horizontalLayout_2;
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
    QVBoxLayout *verticalLayout_3;
    QCheckBox *fCheckBox;
    QCheckBox *pwm1CheckBox;
    QCheckBox *pwm2CheckBox;
    QGroupBox *groupBox_5;
    QVBoxLayout *verticalLayout_8;
    QCheckBox *count1CheckBox;
    QCheckBox *count2CheckBox;
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
    QVBoxLayout *verticalLayout_9;
    QwtPlot *plot1;
    QwtPlot *plot2;
    QwtPlot *plot3;

    void setupUi(QWidget *BoardConsoleClass)
    {
        if (BoardConsoleClass->objectName().isEmpty())
            BoardConsoleClass->setObjectName(QStringLiteral("BoardConsoleClass"));
        BoardConsoleClass->resize(819, 1328);
        horizontalLayout_3 = new QHBoxLayout(BoardConsoleClass);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setSpacing(6);
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        connectGroupBox = new QGroupBox(BoardConsoleClass);
        connectGroupBox->setObjectName(QStringLiteral("connectGroupBox"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(connectGroupBox->sizePolicy().hasHeightForWidth());
        connectGroupBox->setSizePolicy(sizePolicy);
        verticalLayout = new QVBoxLayout(connectGroupBox);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        serialComboBox = new QComboBox(connectGroupBox);
        serialComboBox->setObjectName(QStringLiteral("serialComboBox"));
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


        horizontalLayout->addWidget(connectGroupBox);

        accelgroupBox = new QGroupBox(BoardConsoleClass);
        accelgroupBox->setObjectName(QStringLiteral("accelgroupBox"));
        sizePolicy.setHeightForWidth(accelgroupBox->sizePolicy().hasHeightForWidth());
        accelgroupBox->setSizePolicy(sizePolicy);
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


        horizontalLayout->addWidget(accelgroupBox);

        groupBox_4 = new QGroupBox(BoardConsoleClass);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        sizePolicy.setHeightForWidth(groupBox_4->sizePolicy().hasHeightForWidth());
        groupBox_4->setSizePolicy(sizePolicy);
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


        horizontalLayout->addWidget(groupBox_4);


        verticalLayout_12->addLayout(horizontalLayout);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        researchGroupBox = new QGroupBox(BoardConsoleClass);
        researchGroupBox->setObjectName(QStringLiteral("researchGroupBox"));
        sizePolicy.setHeightForWidth(researchGroupBox->sizePolicy().hasHeightForWidth());
        researchGroupBox->setSizePolicy(sizePolicy);
        verticalLayout_4 = new QVBoxLayout(researchGroupBox);
        verticalLayout_4->setSpacing(6);
        verticalLayout_4->setContentsMargins(11, 11, 11, 11);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        lowpassFilterCheckBox = new QCheckBox(researchGroupBox);
        lowpassFilterCheckBox->setObjectName(QStringLiteral("lowpassFilterCheckBox"));
        lowpassFilterCheckBox->setChecked(true);

        verticalLayout_4->addWidget(lowpassFilterCheckBox);

        turnUselessCheckBox = new QCheckBox(researchGroupBox);
        turnUselessCheckBox->setObjectName(QStringLiteral("turnUselessCheckBox"));

        verticalLayout_4->addWidget(turnUselessCheckBox);

        gyroRecalibrationCheckBox = new QCheckBox(researchGroupBox);
        gyroRecalibrationCheckBox->setObjectName(QStringLiteral("gyroRecalibrationCheckBox"));

        verticalLayout_4->addWidget(gyroRecalibrationCheckBox);

        noResearchRadioButton = new QRadioButton(researchGroupBox);
        noResearchRadioButton->setObjectName(QStringLiteral("noResearchRadioButton"));
        noResearchRadioButton->setChecked(true);

        verticalLayout_4->addWidget(noResearchRadioButton);

        operatorRadioButton = new QRadioButton(researchGroupBox);
        operatorRadioButton->setObjectName(QStringLiteral("operatorRadioButton"));

        verticalLayout_4->addWidget(operatorRadioButton);

        adjustRadioButton = new QRadioButton(researchGroupBox);
        adjustRadioButton->setObjectName(QStringLiteral("adjustRadioButton"));

        verticalLayout_4->addWidget(adjustRadioButton);

        simpleRadioButton = new QRadioButton(researchGroupBox);
        simpleRadioButton->setObjectName(QStringLiteral("simpleRadioButton"));

        verticalLayout_4->addWidget(simpleRadioButton);

        pidRadioButton = new QRadioButton(researchGroupBox);
        pidRadioButton->setObjectName(QStringLiteral("pidRadioButton"));

        verticalLayout_4->addWidget(pidRadioButton);

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

        tranquilityButton = new QPushButton(researchGroupBox);
        tranquilityButton->setObjectName(QStringLiteral("tranquilityButton"));

        verticalLayout_4->addWidget(tranquilityButton);

        tranquilityLineEdit = new QLineEdit(researchGroupBox);
        tranquilityLineEdit->setObjectName(QStringLiteral("tranquilityLineEdit"));

        verticalLayout_4->addWidget(tranquilityLineEdit);

        pwmStepButton = new QPushButton(researchGroupBox);
        pwmStepButton->setObjectName(QStringLiteral("pwmStepButton"));

        verticalLayout_4->addWidget(pwmStepButton);

        pwmStepLineEdit = new QLineEdit(researchGroupBox);
        pwmStepLineEdit->setObjectName(QStringLiteral("pwmStepLineEdit"));

        verticalLayout_4->addWidget(pwmStepLineEdit);

        everyNButton = new QPushButton(researchGroupBox);
        everyNButton->setObjectName(QStringLiteral("everyNButton"));

        verticalLayout_4->addWidget(everyNButton);

        everyNLineEdit = new QLineEdit(researchGroupBox);
        everyNLineEdit->setObjectName(QStringLiteral("everyNLineEdit"));

        verticalLayout_4->addWidget(everyNLineEdit);


        horizontalLayout_6->addWidget(researchGroupBox);

        stabGroupBox = new QGroupBox(BoardConsoleClass);
        stabGroupBox->setObjectName(QStringLiteral("stabGroupBox"));
        sizePolicy.setHeightForWidth(stabGroupBox->sizePolicy().hasHeightForWidth());
        stabGroupBox->setSizePolicy(sizePolicy);
        verticalLayout_2 = new QVBoxLayout(stabGroupBox);
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setContentsMargins(11, 11, 11, 11);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        stopMotorsButton = new QPushButton(stabGroupBox);
        stopMotorsButton->setObjectName(QStringLiteral("stopMotorsButton"));
        sizePolicy.setHeightForWidth(stopMotorsButton->sizePolicy().hasHeightForWidth());
        stopMotorsButton->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(stopMotorsButton);

        calibrButton = new QPushButton(stabGroupBox);
        calibrButton->setObjectName(QStringLiteral("calibrButton"));
        sizePolicy.setHeightForWidth(calibrButton->sizePolicy().hasHeightForWidth());
        calibrButton->setSizePolicy(sizePolicy);

        verticalLayout_2->addWidget(calibrButton);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setSpacing(6);
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label = new QLabel(stabGroupBox);
        label->setObjectName(QStringLiteral("label"));
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        label->setAlignment(Qt::AlignCenter);

        horizontalLayout_4->addWidget(label);

        pwm1SpinBox = new QSpinBox(stabGroupBox);
        pwm1SpinBox->setObjectName(QStringLiteral("pwm1SpinBox"));
        sizePolicy.setHeightForWidth(pwm1SpinBox->sizePolicy().hasHeightForWidth());
        pwm1SpinBox->setSizePolicy(sizePolicy);
        pwm1SpinBox->setMinimum(1000);
        pwm1SpinBox->setMaximum(2000);
        pwm1SpinBox->setValue(1070);

        horizontalLayout_4->addWidget(pwm1SpinBox);


        verticalLayout_2->addLayout(horizontalLayout_4);

        horizontalLayout_11 = new QHBoxLayout();
        horizontalLayout_11->setSpacing(6);
        horizontalLayout_11->setObjectName(QStringLiteral("horizontalLayout_11"));
        label_2 = new QLabel(stabGroupBox);
        label_2->setObjectName(QStringLiteral("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setAlignment(Qt::AlignCenter);

        horizontalLayout_11->addWidget(label_2);

        pwm2SpinBox = new QSpinBox(stabGroupBox);
        pwm2SpinBox->setObjectName(QStringLiteral("pwm2SpinBox"));
        sizePolicy.setHeightForWidth(pwm2SpinBox->sizePolicy().hasHeightForWidth());
        pwm2SpinBox->setSizePolicy(sizePolicy);
        pwm2SpinBox->setMinimum(1000);
        pwm2SpinBox->setMaximum(2000);
        pwm2SpinBox->setValue(2000);

        horizontalLayout_11->addWidget(pwm2SpinBox);


        verticalLayout_2->addLayout(horizontalLayout_11);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setSpacing(6);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_5 = new QLabel(stabGroupBox);
        label_5->setObjectName(QStringLiteral("label_5"));
        sizePolicy.setHeightForWidth(label_5->sizePolicy().hasHeightForWidth());
        label_5->setSizePolicy(sizePolicy);

        horizontalLayout_5->addWidget(label_5);

        pwm1Label = new QLabel(stabGroupBox);
        pwm1Label->setObjectName(QStringLiteral("pwm1Label"));
        sizePolicy.setHeightForWidth(pwm1Label->sizePolicy().hasHeightForWidth());
        pwm1Label->setSizePolicy(sizePolicy);

        horizontalLayout_5->addWidget(pwm1Label);


        verticalLayout_2->addLayout(horizontalLayout_5);

        pwm1Slider = new QSlider(stabGroupBox);
        pwm1Slider->setObjectName(QStringLiteral("pwm1Slider"));
        pwm1Slider->setMinimum(1000);
        pwm1Slider->setMaximum(2000);
        pwm1Slider->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(pwm1Slider);

        horizontalLayout_10 = new QHBoxLayout();
        horizontalLayout_10->setSpacing(6);
        horizontalLayout_10->setObjectName(QStringLiteral("horizontalLayout_10"));
        label_6 = new QLabel(stabGroupBox);
        label_6->setObjectName(QStringLiteral("label_6"));
        sizePolicy.setHeightForWidth(label_6->sizePolicy().hasHeightForWidth());
        label_6->setSizePolicy(sizePolicy);

        horizontalLayout_10->addWidget(label_6);

        pwm2Label = new QLabel(stabGroupBox);
        pwm2Label->setObjectName(QStringLiteral("pwm2Label"));
        sizePolicy.setHeightForWidth(pwm2Label->sizePolicy().hasHeightForWidth());
        pwm2Label->setSizePolicy(sizePolicy);

        horizontalLayout_10->addWidget(pwm2Label);


        verticalLayout_2->addLayout(horizontalLayout_10);

        pwm2Slider = new QSlider(stabGroupBox);
        pwm2Slider->setObjectName(QStringLiteral("pwm2Slider"));
        pwm2Slider->setMinimum(1000);
        pwm2Slider->setMaximum(2000);
        pwm2Slider->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(pwm2Slider);

        maxAngleButton = new QPushButton(stabGroupBox);
        maxAngleButton->setObjectName(QStringLiteral("maxAngleButton"));

        verticalLayout_2->addWidget(maxAngleButton);

        maxAngleLineEdit = new QLineEdit(stabGroupBox);
        maxAngleLineEdit->setObjectName(QStringLiteral("maxAngleLineEdit"));

        verticalLayout_2->addWidget(maxAngleLineEdit);

        maxVelButton = new QPushButton(stabGroupBox);
        maxVelButton->setObjectName(QStringLiteral("maxVelButton"));

        verticalLayout_2->addWidget(maxVelButton);

        maxVelLineEdit = new QLineEdit(stabGroupBox);
        maxVelLineEdit->setObjectName(QStringLiteral("maxVelLineEdit"));

        verticalLayout_2->addWidget(maxVelLineEdit);

        turnoffAngleButton = new QPushButton(stabGroupBox);
        turnoffAngleButton->setObjectName(QStringLiteral("turnoffAngleButton"));

        verticalLayout_2->addWidget(turnoffAngleButton);

        turnoffAngleLineEdit = new QLineEdit(stabGroupBox);
        turnoffAngleLineEdit->setObjectName(QStringLiteral("turnoffAngleLineEdit"));

        verticalLayout_2->addWidget(turnoffAngleLineEdit);

        accelDeviationButton = new QPushButton(stabGroupBox);
        accelDeviationButton->setObjectName(QStringLiteral("accelDeviationButton"));

        verticalLayout_2->addWidget(accelDeviationButton);

        accelDeviationLineEdit = new QLineEdit(stabGroupBox);
        accelDeviationLineEdit->setObjectName(QStringLiteral("accelDeviationLineEdit"));

        verticalLayout_2->addWidget(accelDeviationLineEdit);


        horizontalLayout_6->addWidget(stabGroupBox);


        verticalLayout_12->addLayout(horizontalLayout_6);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        telemetryGroupBox = new QGroupBox(BoardConsoleClass);
        telemetryGroupBox->setObjectName(QStringLiteral("telemetryGroupBox"));
        sizePolicy.setHeightForWidth(telemetryGroupBox->sizePolicy().hasHeightForWidth());
        telemetryGroupBox->setSizePolicy(sizePolicy);
        verticalLayout_11 = new QVBoxLayout(telemetryGroupBox);
        verticalLayout_11->setSpacing(6);
        verticalLayout_11->setContentsMargins(11, 11, 11, 11);
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        saveToFileCheckBox = new QCheckBox(telemetryGroupBox);
        saveToFileCheckBox->setObjectName(QStringLiteral("saveToFileCheckBox"));
        sizePolicy.setHeightForWidth(saveToFileCheckBox->sizePolicy().hasHeightForWidth());
        saveToFileCheckBox->setSizePolicy(sizePolicy);
        saveToFileCheckBox->setChecked(true);

        verticalLayout_11->addWidget(saveToFileCheckBox);

        telemetryToggleButton = new QPushButton(telemetryGroupBox);
        telemetryToggleButton->setObjectName(QStringLiteral("telemetryToggleButton"));
        sizePolicy.setHeightForWidth(telemetryToggleButton->sizePolicy().hasHeightForWidth());
        telemetryToggleButton->setSizePolicy(sizePolicy);

        verticalLayout_11->addWidget(telemetryToggleButton);

        clearTelemetryButton = new QPushButton(telemetryGroupBox);
        clearTelemetryButton->setObjectName(QStringLiteral("clearTelemetryButton"));
        sizePolicy.setHeightForWidth(clearTelemetryButton->sizePolicy().hasHeightForWidth());
        clearTelemetryButton->setSizePolicy(sizePolicy);

        verticalLayout_11->addWidget(clearTelemetryButton);


        horizontalLayout_2->addWidget(telemetryGroupBox);

        groupBox_2 = new QGroupBox(BoardConsoleClass);
        groupBox_2->setObjectName(QStringLiteral("groupBox_2"));
        sizePolicy.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy);
        verticalLayout_7 = new QVBoxLayout(groupBox_2);
        verticalLayout_7->setSpacing(6);
        verticalLayout_7->setContentsMargins(11, 11, 11, 11);
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        angleCheckBox = new QCheckBox(groupBox_2);
        angleCheckBox->setObjectName(QStringLiteral("angleCheckBox"));
        sizePolicy.setHeightForWidth(angleCheckBox->sizePolicy().hasHeightForWidth());
        angleCheckBox->setSizePolicy(sizePolicy);
        angleCheckBox->setChecked(true);

        verticalLayout_7->addWidget(angleCheckBox);

        angVelCheckBox = new QCheckBox(groupBox_2);
        angVelCheckBox->setObjectName(QStringLiteral("angVelCheckBox"));
        angVelCheckBox->setChecked(true);

        verticalLayout_7->addWidget(angVelCheckBox);


        horizontalLayout_2->addWidget(groupBox_2);

        groupBox_3 = new QGroupBox(BoardConsoleClass);
        groupBox_3->setObjectName(QStringLiteral("groupBox_3"));
        sizePolicy.setHeightForWidth(groupBox_3->sizePolicy().hasHeightForWidth());
        groupBox_3->setSizePolicy(sizePolicy);
        verticalLayout_3 = new QVBoxLayout(groupBox_3);
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setContentsMargins(11, 11, 11, 11);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        fCheckBox = new QCheckBox(groupBox_3);
        fCheckBox->setObjectName(QStringLiteral("fCheckBox"));
        sizePolicy.setHeightForWidth(fCheckBox->sizePolicy().hasHeightForWidth());
        fCheckBox->setSizePolicy(sizePolicy);
        fCheckBox->setChecked(false);

        verticalLayout_3->addWidget(fCheckBox);

        pwm1CheckBox = new QCheckBox(groupBox_3);
        pwm1CheckBox->setObjectName(QStringLiteral("pwm1CheckBox"));
        sizePolicy.setHeightForWidth(pwm1CheckBox->sizePolicy().hasHeightForWidth());
        pwm1CheckBox->setSizePolicy(sizePolicy);
        pwm1CheckBox->setChecked(true);

        verticalLayout_3->addWidget(pwm1CheckBox);

        pwm2CheckBox = new QCheckBox(groupBox_3);
        pwm2CheckBox->setObjectName(QStringLiteral("pwm2CheckBox"));
        sizePolicy.setHeightForWidth(pwm2CheckBox->sizePolicy().hasHeightForWidth());
        pwm2CheckBox->setSizePolicy(sizePolicy);
        pwm2CheckBox->setChecked(true);

        verticalLayout_3->addWidget(pwm2CheckBox);


        horizontalLayout_2->addWidget(groupBox_3);

        groupBox_5 = new QGroupBox(BoardConsoleClass);
        groupBox_5->setObjectName(QStringLiteral("groupBox_5"));
        sizePolicy.setHeightForWidth(groupBox_5->sizePolicy().hasHeightForWidth());
        groupBox_5->setSizePolicy(sizePolicy);
        verticalLayout_8 = new QVBoxLayout(groupBox_5);
        verticalLayout_8->setSpacing(6);
        verticalLayout_8->setContentsMargins(11, 11, 11, 11);
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        count1CheckBox = new QCheckBox(groupBox_5);
        count1CheckBox->setObjectName(QStringLiteral("count1CheckBox"));
        sizePolicy.setHeightForWidth(count1CheckBox->sizePolicy().hasHeightForWidth());
        count1CheckBox->setSizePolicy(sizePolicy);
        count1CheckBox->setChecked(true);

        verticalLayout_8->addWidget(count1CheckBox);

        count2CheckBox = new QCheckBox(groupBox_5);
        count2CheckBox->setObjectName(QStringLiteral("count2CheckBox"));
        sizePolicy.setHeightForWidth(count2CheckBox->sizePolicy().hasHeightForWidth());
        count2CheckBox->setSizePolicy(sizePolicy);
        count2CheckBox->setChecked(true);

        verticalLayout_8->addWidget(count2CheckBox);


        horizontalLayout_2->addWidget(groupBox_5);


        verticalLayout_12->addLayout(horizontalLayout_2);

        groupBox = new QGroupBox(BoardConsoleClass);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        sizePolicy.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy);
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
        sizePolicy.setHeightForWidth(iButton->sizePolicy().hasHeightForWidth());
        iButton->setSizePolicy(sizePolicy);

        horizontalLayout_9->addWidget(iButton);

        iLineEdit = new QLineEdit(groupBox);
        iLineEdit->setObjectName(QStringLiteral("iLineEdit"));
        sizePolicy.setHeightForWidth(iLineEdit->sizePolicy().hasHeightForWidth());
        iLineEdit->setSizePolicy(sizePolicy);

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


        verticalLayout_12->addWidget(groupBox);


        horizontalLayout_3->addLayout(verticalLayout_12);

        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setSpacing(6);
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        plot1 = new QwtPlot(BoardConsoleClass);
        plot1->setObjectName(QStringLiteral("plot1"));

        verticalLayout_9->addWidget(plot1);

        plot2 = new QwtPlot(BoardConsoleClass);
        plot2->setObjectName(QStringLiteral("plot2"));

        verticalLayout_9->addWidget(plot2);

        plot3 = new QwtPlot(BoardConsoleClass);
        plot3->setObjectName(QStringLiteral("plot3"));

        verticalLayout_9->addWidget(plot3);


        horizontalLayout_3->addLayout(verticalLayout_9);


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
        turnUselessCheckBox->setText(QApplication::translate("BoardConsoleClass", "Turn Useless", 0));
        gyroRecalibrationCheckBox->setText(QApplication::translate("BoardConsoleClass", "Gyro Recalibration", 0));
        noResearchRadioButton->setText(QApplication::translate("BoardConsoleClass", "No Research", 0));
        operatorRadioButton->setText(QApplication::translate("BoardConsoleClass", "Operator", 0));
        adjustRadioButton->setText(QApplication::translate("BoardConsoleClass", "Adjust", 0));
        simpleRadioButton->setText(QApplication::translate("BoardConsoleClass", "P", 0));
        pidRadioButton->setText(QApplication::translate("BoardConsoleClass", "PID", 0));
        impulseRadioButton->setText(QApplication::translate("BoardConsoleClass", "Impulse", 0));
        stepRadioButton->setText(QApplication::translate("BoardConsoleClass", "Step", 0));
        sineRadioButton->setText(QApplication::translate("BoardConsoleClass", "Sine", 0));
        expRadioButton->setText(QApplication::translate("BoardConsoleClass", "Exp", 0));
        label_3->setText(QApplication::translate("BoardConsoleClass", "Amplitude", 0));
        researchAmplLineEdit->setText(QApplication::translate("BoardConsoleClass", "1.0", 0));
        label_4->setText(QApplication::translate("BoardConsoleClass", "Frequency", 0));
        researchFreqLineEdit->setText(QApplication::translate("BoardConsoleClass", "0.01", 0));
        tranquilityButton->setText(QApplication::translate("BoardConsoleClass", "Tranquility Time (ms)", 0));
        tranquilityLineEdit->setText(QApplication::translate("BoardConsoleClass", "300", 0));
        pwmStepButton->setText(QApplication::translate("BoardConsoleClass", "PWM Step", 0));
        pwmStepLineEdit->setText(QApplication::translate("BoardConsoleClass", "10", 0));
        everyNButton->setText(QApplication::translate("BoardConsoleClass", "Every N", 0));
        everyNLineEdit->setText(QApplication::translate("BoardConsoleClass", "2", 0));
        stabGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Stabilization", 0));
        stopMotorsButton->setText(QApplication::translate("BoardConsoleClass", "Stop Motors", 0));
        calibrButton->setText(QApplication::translate("BoardConsoleClass", "Calibrate", 0));
        label->setText(QApplication::translate("BoardConsoleClass", "Min Pwm", 0));
        label_2->setText(QApplication::translate("BoardConsoleClass", "MaxPwm", 0));
        label_5->setText(QApplication::translate("BoardConsoleClass", "Pwm 1", 0));
        pwm1Label->setText(QApplication::translate("BoardConsoleClass", "1000", 0));
        label_6->setText(QApplication::translate("BoardConsoleClass", "Pwm 2", 0));
        pwm2Label->setText(QApplication::translate("BoardConsoleClass", "1000", 0));
        maxAngleButton->setText(QApplication::translate("BoardConsoleClass", "Max Angle", 0));
        maxAngleLineEdit->setText(QApplication::translate("BoardConsoleClass", "10", 0));
        maxVelButton->setText(QApplication::translate("BoardConsoleClass", "Max Velocity", 0));
        maxVelLineEdit->setText(QApplication::translate("BoardConsoleClass", "30", 0));
        turnoffAngleButton->setText(QApplication::translate("BoardConsoleClass", "TurnOff Angle", 0));
        turnoffAngleLineEdit->setText(QApplication::translate("BoardConsoleClass", "3", 0));
        accelDeviationButton->setText(QApplication::translate("BoardConsoleClass", "Accel Deviation", 0));
        accelDeviationLineEdit->setText(QApplication::translate("BoardConsoleClass", "5", 0));
        telemetryGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Telemetry", 0));
        saveToFileCheckBox->setText(QApplication::translate("BoardConsoleClass", "Save to file", 0));
        telemetryToggleButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
        clearTelemetryButton->setText(QApplication::translate("BoardConsoleClass", "Clear", 0));
        groupBox_2->setTitle(QApplication::translate("BoardConsoleClass", "Plot 1", 0));
        angleCheckBox->setText(QApplication::translate("BoardConsoleClass", "Angle", 0));
        angVelCheckBox->setText(QApplication::translate("BoardConsoleClass", "AngVel", 0));
        groupBox_3->setTitle(QApplication::translate("BoardConsoleClass", "Plot 2", 0));
        fCheckBox->setText(QApplication::translate("BoardConsoleClass", "F", 0));
        pwm1CheckBox->setText(QApplication::translate("BoardConsoleClass", "pwm1", 0));
        pwm2CheckBox->setText(QApplication::translate("BoardConsoleClass", "pwm2", 0));
        groupBox_5->setTitle(QApplication::translate("BoardConsoleClass", "Plot3", 0));
        count1CheckBox->setText(QApplication::translate("BoardConsoleClass", "COUNT1", 0));
        count2CheckBox->setText(QApplication::translate("BoardConsoleClass", "COUNT2", 0));
        groupBox->setTitle(QApplication::translate("BoardConsoleClass", "PID Control", 0));
        pButton->setText(QApplication::translate("BoardConsoleClass", "P", 0));
        iButton->setText(QApplication::translate("BoardConsoleClass", "I", 0));
        dButton->setText(QApplication::translate("BoardConsoleClass", "D", 0));
    } // retranslateUi

};

namespace Ui {
    class BoardConsoleClass: public Ui_BoardConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOARDCONSOLE_H
