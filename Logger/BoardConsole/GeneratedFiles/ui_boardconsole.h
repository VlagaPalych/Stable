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
    QHBoxLayout *horizontalLayout_15;
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_6;
    QGroupBox *connectGroupBox;
    QHBoxLayout *horizontalLayout;
    QComboBox *serialComboBox;
    QPushButton *connectButton;
    QPushButton *programButton;
    QPushButton *rotationButton;
    QHBoxLayout *horizontalLayout_3;
    QGroupBox *researchGroupBox;
    QVBoxLayout *verticalLayout_4;
    QCheckBox *lowpassFilterCheckBox;
    QCheckBox *turnUselessCheckBox;
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
    QVBoxLayout *verticalLayout_10;
    QGroupBox *groupBox_4;
    QHBoxLayout *horizontalLayout_12;
    QwtPlot *plot1;
    QGroupBox *groupBox_7;
    QHBoxLayout *horizontalLayout_14;
    QwtPlot *plot2;
    QGroupBox *groupBox_6;
    QHBoxLayout *horizontalLayout_13;
    QwtPlot *plot3;

    void setupUi(QWidget *BoardConsoleClass)
    {
        if (BoardConsoleClass->objectName().isEmpty())
            BoardConsoleClass->setObjectName(QStringLiteral("BoardConsoleClass"));
        BoardConsoleClass->resize(839, 1427);
        horizontalLayout_15 = new QHBoxLayout(BoardConsoleClass);
        horizontalLayout_15->setSpacing(6);
        horizontalLayout_15->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_15->setObjectName(QStringLiteral("horizontalLayout_15"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setSpacing(6);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        connectGroupBox = new QGroupBox(BoardConsoleClass);
        connectGroupBox->setObjectName(QStringLiteral("connectGroupBox"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(connectGroupBox->sizePolicy().hasHeightForWidth());
        connectGroupBox->setSizePolicy(sizePolicy);
        horizontalLayout = new QHBoxLayout(connectGroupBox);
        horizontalLayout->setSpacing(6);
        horizontalLayout->setContentsMargins(11, 11, 11, 11);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        serialComboBox = new QComboBox(connectGroupBox);
        serialComboBox->setObjectName(QStringLiteral("serialComboBox"));
        sizePolicy.setHeightForWidth(serialComboBox->sizePolicy().hasHeightForWidth());
        serialComboBox->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(serialComboBox);

        connectButton = new QPushButton(connectGroupBox);
        connectButton->setObjectName(QStringLiteral("connectButton"));
        sizePolicy.setHeightForWidth(connectButton->sizePolicy().hasHeightForWidth());
        connectButton->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(connectButton);

        programButton = new QPushButton(connectGroupBox);
        programButton->setObjectName(QStringLiteral("programButton"));
        sizePolicy.setHeightForWidth(programButton->sizePolicy().hasHeightForWidth());
        programButton->setSizePolicy(sizePolicy);

        horizontalLayout->addWidget(programButton);


        horizontalLayout_6->addWidget(connectGroupBox);

        rotationButton = new QPushButton(BoardConsoleClass);
        rotationButton->setObjectName(QStringLiteral("rotationButton"));

        horizontalLayout_6->addWidget(rotationButton);


        verticalLayout->addLayout(horizontalLayout_6);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
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
        sizePolicy.setHeightForWidth(lowpassFilterCheckBox->sizePolicy().hasHeightForWidth());
        lowpassFilterCheckBox->setSizePolicy(sizePolicy);
        lowpassFilterCheckBox->setChecked(true);

        verticalLayout_4->addWidget(lowpassFilterCheckBox);

        turnUselessCheckBox = new QCheckBox(researchGroupBox);
        turnUselessCheckBox->setObjectName(QStringLiteral("turnUselessCheckBox"));
        sizePolicy.setHeightForWidth(turnUselessCheckBox->sizePolicy().hasHeightForWidth());
        turnUselessCheckBox->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(turnUselessCheckBox);

        noResearchRadioButton = new QRadioButton(researchGroupBox);
        noResearchRadioButton->setObjectName(QStringLiteral("noResearchRadioButton"));
        sizePolicy.setHeightForWidth(noResearchRadioButton->sizePolicy().hasHeightForWidth());
        noResearchRadioButton->setSizePolicy(sizePolicy);
        noResearchRadioButton->setChecked(true);

        verticalLayout_4->addWidget(noResearchRadioButton);

        operatorRadioButton = new QRadioButton(researchGroupBox);
        operatorRadioButton->setObjectName(QStringLiteral("operatorRadioButton"));
        sizePolicy.setHeightForWidth(operatorRadioButton->sizePolicy().hasHeightForWidth());
        operatorRadioButton->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(operatorRadioButton);

        adjustRadioButton = new QRadioButton(researchGroupBox);
        adjustRadioButton->setObjectName(QStringLiteral("adjustRadioButton"));
        sizePolicy.setHeightForWidth(adjustRadioButton->sizePolicy().hasHeightForWidth());
        adjustRadioButton->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(adjustRadioButton);

        simpleRadioButton = new QRadioButton(researchGroupBox);
        simpleRadioButton->setObjectName(QStringLiteral("simpleRadioButton"));
        sizePolicy.setHeightForWidth(simpleRadioButton->sizePolicy().hasHeightForWidth());
        simpleRadioButton->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(simpleRadioButton);

        pidRadioButton = new QRadioButton(researchGroupBox);
        pidRadioButton->setObjectName(QStringLiteral("pidRadioButton"));
        sizePolicy.setHeightForWidth(pidRadioButton->sizePolicy().hasHeightForWidth());
        pidRadioButton->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(pidRadioButton);

        impulseRadioButton = new QRadioButton(researchGroupBox);
        impulseRadioButton->setObjectName(QStringLiteral("impulseRadioButton"));
        sizePolicy.setHeightForWidth(impulseRadioButton->sizePolicy().hasHeightForWidth());
        impulseRadioButton->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(impulseRadioButton);

        stepRadioButton = new QRadioButton(researchGroupBox);
        stepRadioButton->setObjectName(QStringLiteral("stepRadioButton"));
        sizePolicy.setHeightForWidth(stepRadioButton->sizePolicy().hasHeightForWidth());
        stepRadioButton->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(stepRadioButton);

        sineRadioButton = new QRadioButton(researchGroupBox);
        sineRadioButton->setObjectName(QStringLiteral("sineRadioButton"));
        sizePolicy.setHeightForWidth(sineRadioButton->sizePolicy().hasHeightForWidth());
        sineRadioButton->setSizePolicy(sizePolicy);

        verticalLayout_4->addWidget(sineRadioButton);

        expRadioButton = new QRadioButton(researchGroupBox);
        expRadioButton->setObjectName(QStringLiteral("expRadioButton"));
        sizePolicy.setHeightForWidth(expRadioButton->sizePolicy().hasHeightForWidth());
        expRadioButton->setSizePolicy(sizePolicy);

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


        horizontalLayout_3->addWidget(researchGroupBox);

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
        sizePolicy.setHeightForWidth(pwm1Slider->sizePolicy().hasHeightForWidth());
        pwm1Slider->setSizePolicy(sizePolicy);
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
        sizePolicy.setHeightForWidth(pwm2Slider->sizePolicy().hasHeightForWidth());
        pwm2Slider->setSizePolicy(sizePolicy);
        pwm2Slider->setMinimum(1000);
        pwm2Slider->setMaximum(2000);
        pwm2Slider->setOrientation(Qt::Horizontal);

        verticalLayout_2->addWidget(pwm2Slider);


        horizontalLayout_3->addWidget(stabGroupBox);


        verticalLayout->addLayout(horizontalLayout_3);

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
        sizePolicy.setHeightForWidth(angVelCheckBox->sizePolicy().hasHeightForWidth());
        angVelCheckBox->setSizePolicy(sizePolicy);
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


        verticalLayout->addLayout(horizontalLayout_2);

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


        verticalLayout->addWidget(groupBox);


        horizontalLayout_15->addLayout(verticalLayout);

        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setSpacing(6);
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        groupBox_4 = new QGroupBox(BoardConsoleClass);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        QSizePolicy sizePolicy1(QSizePolicy::Minimum, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(groupBox_4->sizePolicy().hasHeightForWidth());
        groupBox_4->setSizePolicy(sizePolicy1);
        horizontalLayout_12 = new QHBoxLayout(groupBox_4);
        horizontalLayout_12->setSpacing(6);
        horizontalLayout_12->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_12->setObjectName(QStringLiteral("horizontalLayout_12"));
        plot1 = new QwtPlot(groupBox_4);
        plot1->setObjectName(QStringLiteral("plot1"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(plot1->sizePolicy().hasHeightForWidth());
        plot1->setSizePolicy(sizePolicy2);

        horizontalLayout_12->addWidget(plot1);


        verticalLayout_10->addWidget(groupBox_4);

        groupBox_7 = new QGroupBox(BoardConsoleClass);
        groupBox_7->setObjectName(QStringLiteral("groupBox_7"));
        sizePolicy1.setHeightForWidth(groupBox_7->sizePolicy().hasHeightForWidth());
        groupBox_7->setSizePolicy(sizePolicy1);
        horizontalLayout_14 = new QHBoxLayout(groupBox_7);
        horizontalLayout_14->setSpacing(6);
        horizontalLayout_14->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_14->setObjectName(QStringLiteral("horizontalLayout_14"));
        plot2 = new QwtPlot(groupBox_7);
        plot2->setObjectName(QStringLiteral("plot2"));
        sizePolicy2.setHeightForWidth(plot2->sizePolicy().hasHeightForWidth());
        plot2->setSizePolicy(sizePolicy2);

        horizontalLayout_14->addWidget(plot2);


        verticalLayout_10->addWidget(groupBox_7);

        groupBox_6 = new QGroupBox(BoardConsoleClass);
        groupBox_6->setObjectName(QStringLiteral("groupBox_6"));
        sizePolicy1.setHeightForWidth(groupBox_6->sizePolicy().hasHeightForWidth());
        groupBox_6->setSizePolicy(sizePolicy1);
        horizontalLayout_13 = new QHBoxLayout(groupBox_6);
        horizontalLayout_13->setSpacing(6);
        horizontalLayout_13->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_13->setObjectName(QStringLiteral("horizontalLayout_13"));
        plot3 = new QwtPlot(groupBox_6);
        plot3->setObjectName(QStringLiteral("plot3"));
        sizePolicy2.setHeightForWidth(plot3->sizePolicy().hasHeightForWidth());
        plot3->setSizePolicy(sizePolicy2);

        horizontalLayout_13->addWidget(plot3);


        verticalLayout_10->addWidget(groupBox_6);


        horizontalLayout_15->addLayout(verticalLayout_10);


        retranslateUi(BoardConsoleClass);

        QMetaObject::connectSlotsByName(BoardConsoleClass);
    } // setupUi

    void retranslateUi(QWidget *BoardConsoleClass)
    {
        BoardConsoleClass->setWindowTitle(QApplication::translate("BoardConsoleClass", "BoardConsole", 0));
        connectGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "STM", 0));
        connectButton->setText(QApplication::translate("BoardConsoleClass", "Connect", 0));
        programButton->setText(QApplication::translate("BoardConsoleClass", "Program", 0));
        rotationButton->setText(QApplication::translate("BoardConsoleClass", "Rotation", 0));
        researchGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Research", 0));
        lowpassFilterCheckBox->setText(QApplication::translate("BoardConsoleClass", "lowpass", 0));
        turnUselessCheckBox->setText(QApplication::translate("BoardConsoleClass", "Turn Useless", 0));
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
        stabGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Stabilization", 0));
        stopMotorsButton->setText(QApplication::translate("BoardConsoleClass", "Stop Motors", 0));
        calibrButton->setText(QApplication::translate("BoardConsoleClass", "Calibrate", 0));
        label->setText(QApplication::translate("BoardConsoleClass", "Min Pwm", 0));
        label_2->setText(QApplication::translate("BoardConsoleClass", "MaxPwm", 0));
        label_5->setText(QApplication::translate("BoardConsoleClass", "Pwm 1", 0));
        pwm1Label->setText(QApplication::translate("BoardConsoleClass", "1000", 0));
        label_6->setText(QApplication::translate("BoardConsoleClass", "Pwm 2", 0));
        pwm2Label->setText(QApplication::translate("BoardConsoleClass", "1000", 0));
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
        groupBox_4->setTitle(QApplication::translate("BoardConsoleClass", "Plots", 0));
        groupBox_7->setTitle(QApplication::translate("BoardConsoleClass", "GroupBox", 0));
        groupBox_6->setTitle(QApplication::translate("BoardConsoleClass", "GroupBox", 0));
    } // retranslateUi

};

namespace Ui {
    class BoardConsoleClass: public Ui_BoardConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOARDCONSOLE_H
