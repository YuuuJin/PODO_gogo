/********************************************************************************
** Form generated from reading UI file 'RealWalkDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_REALWALKDIALOG_H
#define UI_REALWALKDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_RealWalkDialog
{
public:
    QCustomPlot *widget;
    QPushButton *PB_WalkReady;
    QTextEdit *TE_Status;
    QPushButton *PB_SingleLog;
    QPushButton *PB_Walking;
    QPushButton *PB_WalkStop;
    QWidget *layoutWidget;
    QVBoxLayout *verticalLayout;
    QLabel *label;
    QFrame *line;
    QLineEdit *LE_StepLength;
    QWidget *layoutWidget1;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_2;
    QFrame *line_2;
    QLineEdit *LE_StepNum;
    QPushButton *PB_WalkingReady;
    QPushButton *PB_SAVE_START;
    QPushButton *PB_SAVE;

    void setupUi(QDialog *RealWalkDialog)
    {
        if (RealWalkDialog->objectName().isEmpty())
            RealWalkDialog->setObjectName(QStringLiteral("RealWalkDialog"));
        RealWalkDialog->resize(687, 625);
        widget = new QCustomPlot(RealWalkDialog);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(120, 60, 531, 221));
        PB_WalkReady = new QPushButton(RealWalkDialog);
        PB_WalkReady->setObjectName(QStringLiteral("PB_WalkReady"));
        PB_WalkReady->setGeometry(QRect(20, 180, 80, 81));
        TE_Status = new QTextEdit(RealWalkDialog);
        TE_Status->setObjectName(QStringLiteral("TE_Status"));
        TE_Status->setGeometry(QRect(20, 80, 81, 81));
        PB_SingleLog = new QPushButton(RealWalkDialog);
        PB_SingleLog->setObjectName(QStringLiteral("PB_SingleLog"));
        PB_SingleLog->setGeometry(QRect(260, 350, 131, 61));
        PB_Walking = new QPushButton(RealWalkDialog);
        PB_Walking->setObjectName(QStringLiteral("PB_Walking"));
        PB_Walking->setGeometry(QRect(50, 420, 131, 61));
        PB_WalkStop = new QPushButton(RealWalkDialog);
        PB_WalkStop->setObjectName(QStringLiteral("PB_WalkStop"));
        PB_WalkStop->setGeometry(QRect(470, 350, 131, 61));
        layoutWidget = new QWidget(RealWalkDialog);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(60, 490, 111, 53));
        verticalLayout = new QVBoxLayout(layoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout->addWidget(label);

        line = new QFrame(layoutWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);

        verticalLayout->addWidget(line);

        LE_StepLength = new QLineEdit(layoutWidget);
        LE_StepLength->setObjectName(QStringLiteral("LE_StepLength"));
        LE_StepLength->setCursorPosition(0);
        LE_StepLength->setAlignment(Qt::AlignCenter);

        verticalLayout->addWidget(LE_StepLength);

        layoutWidget1 = new QWidget(RealWalkDialog);
        layoutWidget1->setObjectName(QStringLiteral("layoutWidget1"));
        layoutWidget1->setGeometry(QRect(270, 420, 111, 53));
        verticalLayout_2 = new QVBoxLayout(layoutWidget1);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(0, 0, 0, 0);
        label_2 = new QLabel(layoutWidget1);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_2->addWidget(label_2);

        line_2 = new QFrame(layoutWidget1);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);

        verticalLayout_2->addWidget(line_2);

        LE_StepNum = new QLineEdit(layoutWidget1);
        LE_StepNum->setObjectName(QStringLiteral("LE_StepNum"));
        LE_StepNum->setAlignment(Qt::AlignCenter);

        verticalLayout_2->addWidget(LE_StepNum);

        PB_WalkingReady = new QPushButton(RealWalkDialog);
        PB_WalkingReady->setObjectName(QStringLiteral("PB_WalkingReady"));
        PB_WalkingReady->setGeometry(QRect(50, 350, 131, 61));
        PB_SAVE_START = new QPushButton(RealWalkDialog);
        PB_SAVE_START->setObjectName(QStringLiteral("PB_SAVE_START"));
        PB_SAVE_START->setGeometry(QRect(510, 520, 80, 22));
        PB_SAVE = new QPushButton(RealWalkDialog);
        PB_SAVE->setObjectName(QStringLiteral("PB_SAVE"));
        PB_SAVE->setGeometry(QRect(510, 550, 80, 22));

        retranslateUi(RealWalkDialog);

        QMetaObject::connectSlotsByName(RealWalkDialog);
    } // setupUi

    void retranslateUi(QDialog *RealWalkDialog)
    {
        RealWalkDialog->setWindowTitle(QApplication::translate("RealWalkDialog", "Dialog", 0));
        PB_WalkReady->setText(QApplication::translate("RealWalkDialog", "WalkReady", 0));
        TE_Status->setHtml(QApplication::translate("RealWalkDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Sans Serif'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\"><span style=\" color:#0800ff;\">Stop</span></p></body></html>", 0));
        PB_SingleLog->setText(QApplication::translate("RealWalkDialog", "SingleLog Start", 0));
        PB_Walking->setText(QApplication::translate("RealWalkDialog", "Walking Start", 0));
        PB_WalkStop->setText(QApplication::translate("RealWalkDialog", "Walking Stop", 0));
        label->setText(QApplication::translate("RealWalkDialog", "<html><head/><body><p align=\"center\">StepLength</p></body></html>", 0));
        LE_StepLength->setText(QApplication::translate("RealWalkDialog", "0.001", 0));
        label_2->setText(QApplication::translate("RealWalkDialog", "<html><head/><body><p align=\"center\">StepNum</p></body></html>", 0));
        LE_StepNum->setText(QApplication::translate("RealWalkDialog", "3", 0));
        PB_WalkingReady->setText(QApplication::translate("RealWalkDialog", "Ready", 0));
        PB_SAVE_START->setText(QApplication::translate("RealWalkDialog", "Save Start", 0));
        PB_SAVE->setText(QApplication::translate("RealWalkDialog", "Save", 0));
    } // retranslateUi

};

namespace Ui {
    class RealWalkDialog: public Ui_RealWalkDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_REALWALKDIALOG_H
