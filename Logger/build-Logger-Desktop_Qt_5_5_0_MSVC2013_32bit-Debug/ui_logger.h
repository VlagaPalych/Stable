/********************************************************************************
** Form generated from reading UI file 'logger.ui'
**
** Created by: Qt User Interface Compiler version 5.5.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOGGER_H
#define UI_LOGGER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_Logger
{
public:
    QVBoxLayout *verticalLayout;
    QHBoxLayout *horizontalLayout_2;
    QComboBox *serialComboBox;
    QPushButton *connectButton;
    QHBoxLayout *horizontalLayout;
    QSpinBox *spinBox;
    QPushButton *stopButton;
    QPushButton *controlButton;
    QPushButton *reloadButton;

    void setupUi(QWidget *Logger)
    {
        if (Logger->objectName().isEmpty())
            Logger->setObjectName(QStringLiteral("Logger"));
        Logger->resize(312, 74);
        verticalLayout = new QVBoxLayout(Logger);
        verticalLayout->setSpacing(6);
        verticalLayout->setContentsMargins(11, 11, 11, 11);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        serialComboBox = new QComboBox(Logger);
        serialComboBox->setObjectName(QStringLiteral("serialComboBox"));

        horizontalLayout_2->addWidget(serialComboBox);

        connectButton = new QPushButton(Logger);
        connectButton->setObjectName(QStringLiteral("connectButton"));

        horizontalLayout_2->addWidget(connectButton);


        verticalLayout->addLayout(horizontalLayout_2);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        spinBox = new QSpinBox(Logger);
        spinBox->setObjectName(QStringLiteral("spinBox"));
        spinBox->setEnabled(false);
        spinBox->setMinimum(1000);
        spinBox->setMaximum(2000);
        spinBox->setSingleStep(50);

        horizontalLayout->addWidget(spinBox);

        stopButton = new QPushButton(Logger);
        stopButton->setObjectName(QStringLiteral("stopButton"));
        stopButton->setEnabled(false);

        horizontalLayout->addWidget(stopButton);

        controlButton = new QPushButton(Logger);
        controlButton->setObjectName(QStringLiteral("controlButton"));
        controlButton->setEnabled(false);

        horizontalLayout->addWidget(controlButton);

        reloadButton = new QPushButton(Logger);
        reloadButton->setObjectName(QStringLiteral("reloadButton"));
        reloadButton->setEnabled(false);

        horizontalLayout->addWidget(reloadButton);


        verticalLayout->addLayout(horizontalLayout);


        retranslateUi(Logger);

        QMetaObject::connectSlotsByName(Logger);
    } // setupUi

    void retranslateUi(QWidget *Logger)
    {
        Logger->setWindowTitle(QApplication::translate("Logger", "Logger", 0));
        connectButton->setText(QApplication::translate("Logger", "Connect", 0));
        stopButton->setText(QApplication::translate("Logger", "Stop", 0));
        controlButton->setText(QApplication::translate("Logger", "Start", 0));
        reloadButton->setText(QApplication::translate("Logger", "Reload", 0));
    } // retranslateUi

};

namespace Ui {
    class Logger: public Ui_Logger {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOGGER_H
