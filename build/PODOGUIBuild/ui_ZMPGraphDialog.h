/********************************************************************************
** Form generated from reading UI file 'ZMPGraphDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ZMPGRAPHDIALOG_H
#define UI_ZMPGRAPHDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPushButton>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_ZMPGraphDialog
{
public:
    QCustomPlot *GRP_ZMPFOOT;
    QCustomPlot *GRP_RT;
    QPushButton *PB_COMPLIANCE_START;
    QPushButton *PB_CLEAR_ZMPFOOT;
    QPushButton *PB_COMPLIANCE_STOP;
    QPushButton *pushButton;
    QPushButton *pushButton_2;
    QCheckBox *CB_GRAPHONOFF;

    void setupUi(QDialog *ZMPGraphDialog)
    {
        if (ZMPGraphDialog->objectName().isEmpty())
            ZMPGraphDialog->setObjectName(QStringLiteral("ZMPGraphDialog"));
        ZMPGraphDialog->resize(833, 696);
        GRP_ZMPFOOT = new QCustomPlot(ZMPGraphDialog);
        GRP_ZMPFOOT->setObjectName(QStringLiteral("GRP_ZMPFOOT"));
        GRP_ZMPFOOT->setGeometry(QRect(90, 310, 400, 400));
        GRP_RT = new QCustomPlot(ZMPGraphDialog);
        GRP_RT->setObjectName(QStringLiteral("GRP_RT"));
        GRP_RT->setGeometry(QRect(40, -20, 731, 321));
        PB_COMPLIANCE_START = new QPushButton(ZMPGraphDialog);
        PB_COMPLIANCE_START->setObjectName(QStringLiteral("PB_COMPLIANCE_START"));
        PB_COMPLIANCE_START->setGeometry(QRect(560, 340, 201, 22));
        PB_CLEAR_ZMPFOOT = new QPushButton(ZMPGraphDialog);
        PB_CLEAR_ZMPFOOT->setObjectName(QStringLiteral("PB_CLEAR_ZMPFOOT"));
        PB_CLEAR_ZMPFOOT->setGeometry(QRect(560, 370, 80, 22));
        PB_COMPLIANCE_STOP = new QPushButton(ZMPGraphDialog);
        PB_COMPLIANCE_STOP->setObjectName(QStringLiteral("PB_COMPLIANCE_STOP"));
        PB_COMPLIANCE_STOP->setGeometry(QRect(560, 410, 201, 22));
        pushButton = new QPushButton(ZMPGraphDialog);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(600, 500, 80, 22));
        pushButton_2 = new QPushButton(ZMPGraphDialog);
        pushButton_2->setObjectName(QStringLiteral("pushButton_2"));
        pushButton_2->setGeometry(QRect(600, 540, 80, 22));
        CB_GRAPHONOFF = new QCheckBox(ZMPGraphDialog);
        CB_GRAPHONOFF->setObjectName(QStringLiteral("CB_GRAPHONOFF"));
        CB_GRAPHONOFF->setGeometry(QRect(500, 310, 85, 20));

        retranslateUi(ZMPGraphDialog);

        QMetaObject::connectSlotsByName(ZMPGraphDialog);
    } // setupUi

    void retranslateUi(QDialog *ZMPGraphDialog)
    {
        ZMPGraphDialog->setWindowTitle(QApplication::translate("ZMPGraphDialog", "Dialog", 0));
        PB_COMPLIANCE_START->setText(QApplication::translate("ZMPGraphDialog", "Start Compliance Control", 0));
        PB_CLEAR_ZMPFOOT->setText(QApplication::translate("ZMPGraphDialog", "Clear Data", 0));
        PB_COMPLIANCE_STOP->setText(QApplication::translate("ZMPGraphDialog", "Stop Compliance Control", 0));
        pushButton->setText(QApplication::translate("ZMPGraphDialog", "Save", 0));
        pushButton_2->setText(QApplication::translate("ZMPGraphDialog", "SaveStart", 0));
        CB_GRAPHONOFF->setText(QApplication::translate("ZMPGraphDialog", "Graph ON", 0));
    } // retranslateUi

};

namespace Ui {
    class ZMPGraphDialog: public Ui_ZMPGraphDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ZMPGRAPHDIALOG_H
