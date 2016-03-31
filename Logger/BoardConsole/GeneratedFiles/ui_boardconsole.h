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
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QListWidget>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QSplitter>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qwt_plot.h"

QT_BEGIN_NAMESPACE

class Ui_BoardConsoleClass
{
public:
    QGridLayout *gridLayout_4;
    QGroupBox *paramsGroupBox;
    QGridLayout *gridLayout;
    QCheckBox *pwm1;
    QCheckBox *eulerZ;
    QCheckBox *f;
    QCheckBox *eulerRateY;
    QCheckBox *pwm2;
    QCheckBox *freq2;
    QCheckBox *freq1;
    QCheckBox *eulerRateZ;
    QCheckBox *compassX;
    QCheckBox *accelX;
    QCheckBox *gyroY;
    QCheckBox *gyroZ;
    QCheckBox *compassY;
    QCheckBox *accelZ;
    QCheckBox *accelY;
    QCheckBox *gyroX;
    QCheckBox *eulerX;
    QCheckBox *eulerRateX;
    QCheckBox *compassZ;
    QCheckBox *eulerY;
    QGroupBox *telemetryGroupBox;
    QHBoxLayout *horizontalLayout_3;
    QPushButton *telemetryToggleButton;
    QPushButton *clearTelemetryButton;
    QCheckBox *saveToFileCheckBox;
    QGroupBox *connectGroupBox;
    QHBoxLayout *horizontalLayout;
    QComboBox *serialComboBox;
    QPushButton *connectButton;
    QPushButton *programButton;
    QGroupBox *groupBox_4;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *rotationButton;
    QRadioButton *mpl;
    QRadioButton *dmp;
    QRadioButton *mine;
    QGroupBox *visualGroupBox;
    QGridLayout *gridLayout_2;
    QListWidget *plot2list;
    QListWidget *plot3list;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QListWidget *plot1list;
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
    QSplitter *splitter;
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
    QGroupBox *researchGroupBox;
    QGridLayout *gridLayout_3;
    QRadioButton *expRadioButton;
    QRadioButton *operatorRadioButton;
    QRadioButton *sineRadioButton;
    QRadioButton *noResearchRadioButton;
    QLineEdit *researchFreqLineEdit;
    QRadioButton *stepRadioButton;
    QLabel *label_4;
    QLineEdit *researchAmplLineEdit;
    QLabel *label_3;
    QRadioButton *pidRadioButton;
    QRadioButton *impulseRadioButton;
    QwtPlot *plot3;
    QwtPlot *plot2;
    QwtPlot *plot1;

    void setupUi(QWidget *BoardConsoleClass)
    {
        if (BoardConsoleClass->objectName().isEmpty())
            BoardConsoleClass->setObjectName(QStringLiteral("BoardConsoleClass"));
        BoardConsoleClass->resize(751, 1202);
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(BoardConsoleClass->sizePolicy().hasHeightForWidth());
        BoardConsoleClass->setSizePolicy(sizePolicy);
        gridLayout_4 = new QGridLayout(BoardConsoleClass);
        gridLayout_4->setSpacing(6);
        gridLayout_4->setContentsMargins(11, 11, 11, 11);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        paramsGroupBox = new QGroupBox(BoardConsoleClass);
        paramsGroupBox->setObjectName(QStringLiteral("paramsGroupBox"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(paramsGroupBox->sizePolicy().hasHeightForWidth());
        paramsGroupBox->setSizePolicy(sizePolicy1);
        gridLayout = new QGridLayout(paramsGroupBox);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        pwm1 = new QCheckBox(paramsGroupBox);
        pwm1->setObjectName(QStringLiteral("pwm1"));
        sizePolicy.setHeightForWidth(pwm1->sizePolicy().hasHeightForWidth());
        pwm1->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pwm1, 6, 0, 1, 1);

        eulerZ = new QCheckBox(paramsGroupBox);
        eulerZ->setObjectName(QStringLiteral("eulerZ"));
        sizePolicy.setHeightForWidth(eulerZ->sizePolicy().hasHeightForWidth());
        eulerZ->setSizePolicy(sizePolicy);

        gridLayout->addWidget(eulerZ, 5, 0, 1, 1);

        f = new QCheckBox(paramsGroupBox);
        f->setObjectName(QStringLiteral("f"));
        sizePolicy.setHeightForWidth(f->sizePolicy().hasHeightForWidth());
        f->setSizePolicy(sizePolicy);

        gridLayout->addWidget(f, 6, 2, 1, 1);

        eulerRateY = new QCheckBox(paramsGroupBox);
        eulerRateY->setObjectName(QStringLiteral("eulerRateY"));
        sizePolicy.setHeightForWidth(eulerRateY->sizePolicy().hasHeightForWidth());
        eulerRateY->setSizePolicy(sizePolicy);

        gridLayout->addWidget(eulerRateY, 4, 1, 1, 1);

        pwm2 = new QCheckBox(paramsGroupBox);
        pwm2->setObjectName(QStringLiteral("pwm2"));
        sizePolicy.setHeightForWidth(pwm2->sizePolicy().hasHeightForWidth());
        pwm2->setSizePolicy(sizePolicy);

        gridLayout->addWidget(pwm2, 7, 0, 1, 1);

        freq2 = new QCheckBox(paramsGroupBox);
        freq2->setObjectName(QStringLiteral("freq2"));
        sizePolicy.setHeightForWidth(freq2->sizePolicy().hasHeightForWidth());
        freq2->setSizePolicy(sizePolicy);

        gridLayout->addWidget(freq2, 7, 1, 1, 1);

        freq1 = new QCheckBox(paramsGroupBox);
        freq1->setObjectName(QStringLiteral("freq1"));
        sizePolicy.setHeightForWidth(freq1->sizePolicy().hasHeightForWidth());
        freq1->setSizePolicy(sizePolicy);

        gridLayout->addWidget(freq1, 6, 1, 1, 1);

        eulerRateZ = new QCheckBox(paramsGroupBox);
        eulerRateZ->setObjectName(QStringLiteral("eulerRateZ"));
        sizePolicy.setHeightForWidth(eulerRateZ->sizePolicy().hasHeightForWidth());
        eulerRateZ->setSizePolicy(sizePolicy);

        gridLayout->addWidget(eulerRateZ, 5, 1, 1, 1);

        compassX = new QCheckBox(paramsGroupBox);
        compassX->setObjectName(QStringLiteral("compassX"));
        sizePolicy.setHeightForWidth(compassX->sizePolicy().hasHeightForWidth());
        compassX->setSizePolicy(sizePolicy);

        gridLayout->addWidget(compassX, 0, 2, 1, 1);

        accelX = new QCheckBox(paramsGroupBox);
        accelX->setObjectName(QStringLiteral("accelX"));
        sizePolicy.setHeightForWidth(accelX->sizePolicy().hasHeightForWidth());
        accelX->setSizePolicy(sizePolicy);

        gridLayout->addWidget(accelX, 0, 0, 1, 1);

        gyroY = new QCheckBox(paramsGroupBox);
        gyroY->setObjectName(QStringLiteral("gyroY"));
        sizePolicy.setHeightForWidth(gyroY->sizePolicy().hasHeightForWidth());
        gyroY->setSizePolicy(sizePolicy);

        gridLayout->addWidget(gyroY, 1, 1, 1, 1);

        gyroZ = new QCheckBox(paramsGroupBox);
        gyroZ->setObjectName(QStringLiteral("gyroZ"));
        sizePolicy.setHeightForWidth(gyroZ->sizePolicy().hasHeightForWidth());
        gyroZ->setSizePolicy(sizePolicy);

        gridLayout->addWidget(gyroZ, 2, 1, 1, 1);

        compassY = new QCheckBox(paramsGroupBox);
        compassY->setObjectName(QStringLiteral("compassY"));
        sizePolicy.setHeightForWidth(compassY->sizePolicy().hasHeightForWidth());
        compassY->setSizePolicy(sizePolicy);

        gridLayout->addWidget(compassY, 1, 2, 1, 1);

        accelZ = new QCheckBox(paramsGroupBox);
        accelZ->setObjectName(QStringLiteral("accelZ"));
        sizePolicy.setHeightForWidth(accelZ->sizePolicy().hasHeightForWidth());
        accelZ->setSizePolicy(sizePolicy);

        gridLayout->addWidget(accelZ, 2, 0, 1, 1);

        accelY = new QCheckBox(paramsGroupBox);
        accelY->setObjectName(QStringLiteral("accelY"));
        sizePolicy.setHeightForWidth(accelY->sizePolicy().hasHeightForWidth());
        accelY->setSizePolicy(sizePolicy);

        gridLayout->addWidget(accelY, 1, 0, 1, 1);

        gyroX = new QCheckBox(paramsGroupBox);
        gyroX->setObjectName(QStringLiteral("gyroX"));
        sizePolicy.setHeightForWidth(gyroX->sizePolicy().hasHeightForWidth());
        gyroX->setSizePolicy(sizePolicy);

        gridLayout->addWidget(gyroX, 0, 1, 1, 1);

        eulerX = new QCheckBox(paramsGroupBox);
        eulerX->setObjectName(QStringLiteral("eulerX"));
        sizePolicy.setHeightForWidth(eulerX->sizePolicy().hasHeightForWidth());
        eulerX->setSizePolicy(sizePolicy);

        gridLayout->addWidget(eulerX, 3, 0, 1, 1);

        eulerRateX = new QCheckBox(paramsGroupBox);
        eulerRateX->setObjectName(QStringLiteral("eulerRateX"));
        sizePolicy.setHeightForWidth(eulerRateX->sizePolicy().hasHeightForWidth());
        eulerRateX->setSizePolicy(sizePolicy);

        gridLayout->addWidget(eulerRateX, 3, 1, 1, 1);

        compassZ = new QCheckBox(paramsGroupBox);
        compassZ->setObjectName(QStringLiteral("compassZ"));
        sizePolicy.setHeightForWidth(compassZ->sizePolicy().hasHeightForWidth());
        compassZ->setSizePolicy(sizePolicy);

        gridLayout->addWidget(compassZ, 2, 2, 1, 1);

        eulerY = new QCheckBox(paramsGroupBox);
        eulerY->setObjectName(QStringLiteral("eulerY"));
        sizePolicy.setHeightForWidth(eulerY->sizePolicy().hasHeightForWidth());
        eulerY->setSizePolicy(sizePolicy);

        gridLayout->addWidget(eulerY, 4, 0, 1, 1);


        gridLayout_4->addWidget(paramsGroupBox, 2, 0, 2, 1);

        telemetryGroupBox = new QGroupBox(BoardConsoleClass);
        telemetryGroupBox->setObjectName(QStringLiteral("telemetryGroupBox"));
        sizePolicy1.setHeightForWidth(telemetryGroupBox->sizePolicy().hasHeightForWidth());
        telemetryGroupBox->setSizePolicy(sizePolicy1);
        horizontalLayout_3 = new QHBoxLayout(telemetryGroupBox);
        horizontalLayout_3->setSpacing(6);
        horizontalLayout_3->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        telemetryToggleButton = new QPushButton(telemetryGroupBox);
        telemetryToggleButton->setObjectName(QStringLiteral("telemetryToggleButton"));
        sizePolicy.setHeightForWidth(telemetryToggleButton->sizePolicy().hasHeightForWidth());
        telemetryToggleButton->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(telemetryToggleButton);

        clearTelemetryButton = new QPushButton(telemetryGroupBox);
        clearTelemetryButton->setObjectName(QStringLiteral("clearTelemetryButton"));
        sizePolicy.setHeightForWidth(clearTelemetryButton->sizePolicy().hasHeightForWidth());
        clearTelemetryButton->setSizePolicy(sizePolicy);

        horizontalLayout_3->addWidget(clearTelemetryButton);

        saveToFileCheckBox = new QCheckBox(telemetryGroupBox);
        saveToFileCheckBox->setObjectName(QStringLiteral("saveToFileCheckBox"));
        sizePolicy.setHeightForWidth(saveToFileCheckBox->sizePolicy().hasHeightForWidth());
        saveToFileCheckBox->setSizePolicy(sizePolicy);
        saveToFileCheckBox->setChecked(true);

        horizontalLayout_3->addWidget(saveToFileCheckBox);


        gridLayout_4->addWidget(telemetryGroupBox, 1, 0, 1, 1);

        connectGroupBox = new QGroupBox(BoardConsoleClass);
        connectGroupBox->setObjectName(QStringLiteral("connectGroupBox"));
        sizePolicy1.setHeightForWidth(connectGroupBox->sizePolicy().hasHeightForWidth());
        connectGroupBox->setSizePolicy(sizePolicy1);
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


        gridLayout_4->addWidget(connectGroupBox, 0, 0, 1, 1);

        groupBox_4 = new QGroupBox(BoardConsoleClass);
        groupBox_4->setObjectName(QStringLiteral("groupBox_4"));
        sizePolicy1.setHeightForWidth(groupBox_4->sizePolicy().hasHeightForWidth());
        groupBox_4->setSizePolicy(sizePolicy1);
        horizontalLayout_2 = new QHBoxLayout(groupBox_4);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        rotationButton = new QPushButton(groupBox_4);
        rotationButton->setObjectName(QStringLiteral("rotationButton"));
        sizePolicy.setHeightForWidth(rotationButton->sizePolicy().hasHeightForWidth());
        rotationButton->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(rotationButton);

        mpl = new QRadioButton(groupBox_4);
        mpl->setObjectName(QStringLiteral("mpl"));
        sizePolicy.setHeightForWidth(mpl->sizePolicy().hasHeightForWidth());
        mpl->setSizePolicy(sizePolicy);
        mpl->setChecked(true);

        horizontalLayout_2->addWidget(mpl);

        dmp = new QRadioButton(groupBox_4);
        dmp->setObjectName(QStringLiteral("dmp"));
        sizePolicy.setHeightForWidth(dmp->sizePolicy().hasHeightForWidth());
        dmp->setSizePolicy(sizePolicy);
        dmp->setChecked(false);

        horizontalLayout_2->addWidget(dmp);

        mine = new QRadioButton(groupBox_4);
        mine->setObjectName(QStringLiteral("mine"));
        sizePolicy.setHeightForWidth(mine->sizePolicy().hasHeightForWidth());
        mine->setSizePolicy(sizePolicy);

        horizontalLayout_2->addWidget(mine);


        gridLayout_4->addWidget(groupBox_4, 5, 0, 1, 1);

        visualGroupBox = new QGroupBox(BoardConsoleClass);
        visualGroupBox->setObjectName(QStringLiteral("visualGroupBox"));
        QSizePolicy sizePolicy2(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(visualGroupBox->sizePolicy().hasHeightForWidth());
        visualGroupBox->setSizePolicy(sizePolicy2);
        visualGroupBox->setMaximumSize(QSize(280, 120));
        gridLayout_2 = new QGridLayout(visualGroupBox);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        plot2list = new QListWidget(visualGroupBox);
        plot2list->setObjectName(QStringLiteral("plot2list"));
        sizePolicy2.setHeightForWidth(plot2list->sizePolicy().hasHeightForWidth());
        plot2list->setSizePolicy(sizePolicy2);
        plot2list->setDragEnabled(true);
        plot2list->setDragDropOverwriteMode(true);
        plot2list->setDragDropMode(QAbstractItemView::DragDrop);
        plot2list->setDefaultDropAction(Qt::MoveAction);
        plot2list->setSelectionMode(QAbstractItemView::MultiSelection);
        plot2list->setSelectionRectVisible(true);

        gridLayout_2->addWidget(plot2list, 1, 1, 1, 1);

        plot3list = new QListWidget(visualGroupBox);
        plot3list->setObjectName(QStringLiteral("plot3list"));
        sizePolicy2.setHeightForWidth(plot3list->sizePolicy().hasHeightForWidth());
        plot3list->setSizePolicy(sizePolicy2);
        plot3list->setDragEnabled(true);
        plot3list->setDragDropOverwriteMode(true);
        plot3list->setDragDropMode(QAbstractItemView::DragDrop);
        plot3list->setDefaultDropAction(Qt::MoveAction);
        plot3list->setSelectionMode(QAbstractItemView::MultiSelection);
        plot3list->setSelectionRectVisible(true);

        gridLayout_2->addWidget(plot3list, 1, 2, 1, 1);

        label_7 = new QLabel(visualGroupBox);
        label_7->setObjectName(QStringLiteral("label_7"));
        sizePolicy.setHeightForWidth(label_7->sizePolicy().hasHeightForWidth());
        label_7->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(label_7, 0, 0, 1, 1);

        label_8 = new QLabel(visualGroupBox);
        label_8->setObjectName(QStringLiteral("label_8"));
        sizePolicy.setHeightForWidth(label_8->sizePolicy().hasHeightForWidth());
        label_8->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(label_8, 0, 1, 1, 1);

        label_9 = new QLabel(visualGroupBox);
        label_9->setObjectName(QStringLiteral("label_9"));
        sizePolicy.setHeightForWidth(label_9->sizePolicy().hasHeightForWidth());
        label_9->setSizePolicy(sizePolicy);

        gridLayout_2->addWidget(label_9, 0, 2, 1, 1);

        plot1list = new QListWidget(visualGroupBox);
        plot1list->setObjectName(QStringLiteral("plot1list"));
        sizePolicy2.setHeightForWidth(plot1list->sizePolicy().hasHeightForWidth());
        plot1list->setSizePolicy(sizePolicy2);
        plot1list->setDragEnabled(true);
        plot1list->setDragDropOverwriteMode(true);
        plot1list->setDragDropMode(QAbstractItemView::DragDrop);
        plot1list->setDefaultDropAction(Qt::MoveAction);
        plot1list->setSelectionMode(QAbstractItemView::MultiSelection);
        plot1list->setSelectionRectVisible(true);

        gridLayout_2->addWidget(plot1list, 1, 0, 1, 1);


        gridLayout_4->addWidget(visualGroupBox, 4, 0, 1, 1);

        groupBox = new QGroupBox(BoardConsoleClass);
        groupBox->setObjectName(QStringLiteral("groupBox"));
        sizePolicy1.setHeightForWidth(groupBox->sizePolicy().hasHeightForWidth());
        groupBox->setSizePolicy(sizePolicy1);
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


        gridLayout_4->addWidget(groupBox, 6, 0, 1, 1);

        splitter = new QSplitter(BoardConsoleClass);
        splitter->setObjectName(QStringLiteral("splitter"));
        sizePolicy2.setHeightForWidth(splitter->sizePolicy().hasHeightForWidth());
        splitter->setSizePolicy(sizePolicy2);
        splitter->setOrientation(Qt::Horizontal);
        stabGroupBox = new QGroupBox(splitter);
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

        splitter->addWidget(stabGroupBox);
        researchGroupBox = new QGroupBox(splitter);
        researchGroupBox->setObjectName(QStringLiteral("researchGroupBox"));
        sizePolicy.setHeightForWidth(researchGroupBox->sizePolicy().hasHeightForWidth());
        researchGroupBox->setSizePolicy(sizePolicy);
        gridLayout_3 = new QGridLayout(researchGroupBox);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        expRadioButton = new QRadioButton(researchGroupBox);
        expRadioButton->setObjectName(QStringLiteral("expRadioButton"));
        sizePolicy.setHeightForWidth(expRadioButton->sizePolicy().hasHeightForWidth());
        expRadioButton->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(expRadioButton, 6, 0, 1, 1);

        operatorRadioButton = new QRadioButton(researchGroupBox);
        operatorRadioButton->setObjectName(QStringLiteral("operatorRadioButton"));
        sizePolicy.setHeightForWidth(operatorRadioButton->sizePolicy().hasHeightForWidth());
        operatorRadioButton->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(operatorRadioButton, 1, 0, 1, 1);

        sineRadioButton = new QRadioButton(researchGroupBox);
        sineRadioButton->setObjectName(QStringLiteral("sineRadioButton"));
        sizePolicy.setHeightForWidth(sineRadioButton->sizePolicy().hasHeightForWidth());
        sineRadioButton->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(sineRadioButton, 5, 0, 1, 1);

        noResearchRadioButton = new QRadioButton(researchGroupBox);
        noResearchRadioButton->setObjectName(QStringLiteral("noResearchRadioButton"));
        sizePolicy.setHeightForWidth(noResearchRadioButton->sizePolicy().hasHeightForWidth());
        noResearchRadioButton->setSizePolicy(sizePolicy);
        noResearchRadioButton->setChecked(true);

        gridLayout_3->addWidget(noResearchRadioButton, 0, 0, 1, 1);

        researchFreqLineEdit = new QLineEdit(researchGroupBox);
        researchFreqLineEdit->setObjectName(QStringLiteral("researchFreqLineEdit"));
        sizePolicy.setHeightForWidth(researchFreqLineEdit->sizePolicy().hasHeightForWidth());
        researchFreqLineEdit->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(researchFreqLineEdit, 10, 0, 1, 1);

        stepRadioButton = new QRadioButton(researchGroupBox);
        stepRadioButton->setObjectName(QStringLiteral("stepRadioButton"));
        sizePolicy.setHeightForWidth(stepRadioButton->sizePolicy().hasHeightForWidth());
        stepRadioButton->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(stepRadioButton, 4, 0, 1, 1);

        label_4 = new QLabel(researchGroupBox);
        label_4->setObjectName(QStringLiteral("label_4"));
        sizePolicy.setHeightForWidth(label_4->sizePolicy().hasHeightForWidth());
        label_4->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(label_4, 9, 0, 1, 1);

        researchAmplLineEdit = new QLineEdit(researchGroupBox);
        researchAmplLineEdit->setObjectName(QStringLiteral("researchAmplLineEdit"));
        sizePolicy.setHeightForWidth(researchAmplLineEdit->sizePolicy().hasHeightForWidth());
        researchAmplLineEdit->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(researchAmplLineEdit, 8, 0, 1, 1);

        label_3 = new QLabel(researchGroupBox);
        label_3->setObjectName(QStringLiteral("label_3"));
        sizePolicy.setHeightForWidth(label_3->sizePolicy().hasHeightForWidth());
        label_3->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(label_3, 7, 0, 1, 1);

        pidRadioButton = new QRadioButton(researchGroupBox);
        pidRadioButton->setObjectName(QStringLiteral("pidRadioButton"));
        sizePolicy.setHeightForWidth(pidRadioButton->sizePolicy().hasHeightForWidth());
        pidRadioButton->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(pidRadioButton, 2, 0, 1, 1);

        impulseRadioButton = new QRadioButton(researchGroupBox);
        impulseRadioButton->setObjectName(QStringLiteral("impulseRadioButton"));
        sizePolicy.setHeightForWidth(impulseRadioButton->sizePolicy().hasHeightForWidth());
        impulseRadioButton->setSizePolicy(sizePolicy);

        gridLayout_3->addWidget(impulseRadioButton, 3, 0, 1, 1);

        splitter->addWidget(researchGroupBox);

        gridLayout_4->addWidget(splitter, 7, 0, 1, 1);

        plot3 = new QwtPlot(BoardConsoleClass);
        plot3->setObjectName(QStringLiteral("plot3"));
        QSizePolicy sizePolicy3(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(10);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(plot3->sizePolicy().hasHeightForWidth());
        plot3->setSizePolicy(sizePolicy3);

        gridLayout_4->addWidget(plot3, 7, 1, 1, 1);

        plot2 = new QwtPlot(BoardConsoleClass);
        plot2->setObjectName(QStringLiteral("plot2"));
        sizePolicy3.setHeightForWidth(plot2->sizePolicy().hasHeightForWidth());
        plot2->setSizePolicy(sizePolicy3);

        gridLayout_4->addWidget(plot2, 4, 1, 3, 1);

        plot1 = new QwtPlot(BoardConsoleClass);
        plot1->setObjectName(QStringLiteral("plot1"));
        sizePolicy3.setHeightForWidth(plot1->sizePolicy().hasHeightForWidth());
        plot1->setSizePolicy(sizePolicy3);

        gridLayout_4->addWidget(plot1, 0, 1, 4, 1);


        retranslateUi(BoardConsoleClass);

        QMetaObject::connectSlotsByName(BoardConsoleClass);
    } // setupUi

    void retranslateUi(QWidget *BoardConsoleClass)
    {
        BoardConsoleClass->setWindowTitle(QApplication::translate("BoardConsoleClass", "BoardConsole", 0));
        paramsGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Parameters", 0));
        pwm1->setText(QApplication::translate("BoardConsoleClass", "pwm 1", 0));
        eulerZ->setText(QApplication::translate("BoardConsoleClass", "Euler Z", 0));
        f->setText(QApplication::translate("BoardConsoleClass", "F", 0));
        eulerRateY->setText(QApplication::translate("BoardConsoleClass", "EulerRate Y", 0));
        pwm2->setText(QApplication::translate("BoardConsoleClass", "pwm 2", 0));
        freq2->setText(QApplication::translate("BoardConsoleClass", "Freq 2", 0));
        freq1->setText(QApplication::translate("BoardConsoleClass", "Freq 1", 0));
        eulerRateZ->setText(QApplication::translate("BoardConsoleClass", "EulerRate Z", 0));
        compassX->setText(QApplication::translate("BoardConsoleClass", "Compass X", 0));
        accelX->setText(QApplication::translate("BoardConsoleClass", "Accel X", 0));
        gyroY->setText(QApplication::translate("BoardConsoleClass", "Gyro Y", 0));
        gyroZ->setText(QApplication::translate("BoardConsoleClass", "Gyro Z", 0));
        compassY->setText(QApplication::translate("BoardConsoleClass", "Compass Y", 0));
        accelZ->setText(QApplication::translate("BoardConsoleClass", "Accel Z", 0));
        accelY->setText(QApplication::translate("BoardConsoleClass", "Accel Y", 0));
        gyroX->setText(QApplication::translate("BoardConsoleClass", "Gyro X", 0));
        eulerX->setText(QApplication::translate("BoardConsoleClass", "Euler X", 0));
        eulerRateX->setText(QApplication::translate("BoardConsoleClass", "EulerRate X", 0));
        compassZ->setText(QApplication::translate("BoardConsoleClass", "Compass Z", 0));
        eulerY->setText(QApplication::translate("BoardConsoleClass", "Euler Y", 0));
        telemetryGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Telemetry", 0));
        telemetryToggleButton->setText(QApplication::translate("BoardConsoleClass", "Start", 0));
        clearTelemetryButton->setText(QApplication::translate("BoardConsoleClass", "Clear", 0));
        saveToFileCheckBox->setText(QApplication::translate("BoardConsoleClass", "Save to file", 0));
        connectGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "STM", 0));
        connectButton->setText(QApplication::translate("BoardConsoleClass", "Connect", 0));
        programButton->setText(QApplication::translate("BoardConsoleClass", "Program", 0));
        groupBox_4->setTitle(QApplication::translate("BoardConsoleClass", "Orientation", 0));
        rotationButton->setText(QApplication::translate("BoardConsoleClass", "Widget", 0));
        mpl->setText(QApplication::translate("BoardConsoleClass", "MPL", 0));
        dmp->setText(QApplication::translate("BoardConsoleClass", "DMP", 0));
        mine->setText(QApplication::translate("BoardConsoleClass", "Mine", 0));
        visualGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Visualization", 0));
        label_7->setText(QApplication::translate("BoardConsoleClass", "Plot 1", 0));
        label_8->setText(QApplication::translate("BoardConsoleClass", "Plot 2", 0));
        label_9->setText(QApplication::translate("BoardConsoleClass", "Plot 3", 0));
        groupBox->setTitle(QApplication::translate("BoardConsoleClass", "PID Control", 0));
        pButton->setText(QApplication::translate("BoardConsoleClass", "P", 0));
        iButton->setText(QApplication::translate("BoardConsoleClass", "I", 0));
        dButton->setText(QApplication::translate("BoardConsoleClass", "D", 0));
        stabGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Stabilization", 0));
        stopMotorsButton->setText(QApplication::translate("BoardConsoleClass", "Stop Motors", 0));
        calibrButton->setText(QApplication::translate("BoardConsoleClass", "Calibrate", 0));
        label->setText(QApplication::translate("BoardConsoleClass", "Min Pwm", 0));
        label_2->setText(QApplication::translate("BoardConsoleClass", "MaxPwm", 0));
        label_5->setText(QApplication::translate("BoardConsoleClass", "Pwm 1", 0));
        pwm1Label->setText(QApplication::translate("BoardConsoleClass", "1000", 0));
        label_6->setText(QApplication::translate("BoardConsoleClass", "Pwm 2", 0));
        pwm2Label->setText(QApplication::translate("BoardConsoleClass", "1000", 0));
        researchGroupBox->setTitle(QApplication::translate("BoardConsoleClass", "Research", 0));
        expRadioButton->setText(QApplication::translate("BoardConsoleClass", "Exp", 0));
        operatorRadioButton->setText(QApplication::translate("BoardConsoleClass", "Operator", 0));
        sineRadioButton->setText(QApplication::translate("BoardConsoleClass", "Sine", 0));
        noResearchRadioButton->setText(QApplication::translate("BoardConsoleClass", "No Research", 0));
        researchFreqLineEdit->setText(QApplication::translate("BoardConsoleClass", "0.01", 0));
        stepRadioButton->setText(QApplication::translate("BoardConsoleClass", "Step", 0));
        label_4->setText(QApplication::translate("BoardConsoleClass", "Frequency", 0));
        researchAmplLineEdit->setText(QApplication::translate("BoardConsoleClass", "1.0", 0));
        label_3->setText(QApplication::translate("BoardConsoleClass", "Amplitude", 0));
        pidRadioButton->setText(QApplication::translate("BoardConsoleClass", "PID", 0));
        impulseRadioButton->setText(QApplication::translate("BoardConsoleClass", "Impulse", 0));
    } // retranslateUi

};

namespace Ui {
    class BoardConsoleClass: public Ui_BoardConsoleClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BOARDCONSOLE_H
