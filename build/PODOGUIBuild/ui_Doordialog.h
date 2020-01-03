/********************************************************************************
** Form generated from reading UI file 'Doordialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DOORDIALOG_H
#define UI_DOORDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_DoorDialog
{
public:
    QFrame *FR_APPROACH;
    QPushButton *BTN_DATA_SAVE;
    QPushButton *BTN_SAVE_START;
    QPushButton *BTN_Walk_Ready;
    QPushButton *BTN_FORWARD;
    QPushButton *BTN_CW;
    QPushButton *BTN_LEFT;
    QPushButton *BTN_RIGHT;
    QPushButton *BTN_CCW;
    QWidget *verticalLayoutWidget_3;
    QVBoxLayout *verticalLayout_3;
    QPushButton *PB_READY;
    QPushButton *PB_START;
    QPushButton *PB_STOP;
    QWidget *verticalLayoutWidget_4;
    QVBoxLayout *verticalLayout_4;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QLineEdit *LE_STEP_NUM;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *LE_STEP_LENGTH;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_4;
    QLineEdit *LE_STEP_OFFSET;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_3;
    QLineEdit *LE_STEP_ANGLE;
    QRadioButton *MODE_NORMAL;
    QRadioButton *MODE_LIFTBOX;
    QRadioButton *MODE_DOOR;
    QCustomPlot *GRP_ZMPFOOT;
    QCheckBox *CB_GRAPHONOFF;
    QPushButton *PB_CLEAR_ZMPFOOT;
    QWidget *widget;
    QVBoxLayout *verticalLayout;
    QPushButton *PB_COMPLIANCE_START;
    QPushButton *PB_PUSHDOOR;
    QPushButton *PB_LEANEDFORWARD;
    QWidget *widget1;
    QHBoxLayout *horizontalLayout_5;
    QPushButton *PB_LEANED_B;
    QPushButton *PB_LEANED_F;
    QVBoxLayout *verticalLayout_2;
    QLabel *label_5;
    QLineEdit *LE_LEANEDF_COMX;
    QButtonGroup *WALK_MODE;

    void setupUi(QDialog *DoorDialog)
    {
        if (DoorDialog->objectName().isEmpty())
            DoorDialog->setObjectName(QStringLiteral("DoorDialog"));
        DoorDialog->resize(833, 696);
        FR_APPROACH = new QFrame(DoorDialog);
        FR_APPROACH->setObjectName(QStringLiteral("FR_APPROACH"));
        FR_APPROACH->setGeometry(QRect(20, 0, 491, 271));
        FR_APPROACH->setFrameShape(QFrame::StyledPanel);
        FR_APPROACH->setFrameShadow(QFrame::Raised);
        BTN_DATA_SAVE = new QPushButton(FR_APPROACH);
        BTN_DATA_SAVE->setObjectName(QStringLiteral("BTN_DATA_SAVE"));
        BTN_DATA_SAVE->setGeometry(QRect(170, 230, 81, 31));
        BTN_SAVE_START = new QPushButton(FR_APPROACH);
        BTN_SAVE_START->setObjectName(QStringLiteral("BTN_SAVE_START"));
        BTN_SAVE_START->setGeometry(QRect(170, 200, 81, 31));
        BTN_Walk_Ready = new QPushButton(FR_APPROACH);
        BTN_Walk_Ready->setObjectName(QStringLiteral("BTN_Walk_Ready"));
        BTN_Walk_Ready->setGeometry(QRect(10, 20, 101, 41));
        BTN_FORWARD = new QPushButton(FR_APPROACH);
        BTN_FORWARD->setObjectName(QStringLiteral("BTN_FORWARD"));
        BTN_FORWARD->setGeometry(QRect(10, 70, 101, 41));
        BTN_CW = new QPushButton(FR_APPROACH);
        BTN_CW->setObjectName(QStringLiteral("BTN_CW"));
        BTN_CW->setGeometry(QRect(10, 200, 101, 31));
        BTN_LEFT = new QPushButton(FR_APPROACH);
        BTN_LEFT->setObjectName(QStringLiteral("BTN_LEFT"));
        BTN_LEFT->setGeometry(QRect(10, 120, 51, 31));
        BTN_RIGHT = new QPushButton(FR_APPROACH);
        BTN_RIGHT->setObjectName(QStringLiteral("BTN_RIGHT"));
        BTN_RIGHT->setGeometry(QRect(60, 120, 51, 31));
        BTN_CCW = new QPushButton(FR_APPROACH);
        BTN_CCW->setObjectName(QStringLiteral("BTN_CCW"));
        BTN_CCW->setGeometry(QRect(10, 160, 101, 31));
        verticalLayoutWidget_3 = new QWidget(FR_APPROACH);
        verticalLayoutWidget_3->setObjectName(QStringLiteral("verticalLayoutWidget_3"));
        verticalLayoutWidget_3->setGeometry(QRect(370, 150, 101, 80));
        verticalLayout_3 = new QVBoxLayout(verticalLayoutWidget_3);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        verticalLayout_3->setContentsMargins(0, 0, 0, 0);
        PB_READY = new QPushButton(verticalLayoutWidget_3);
        PB_READY->setObjectName(QStringLiteral("PB_READY"));

        verticalLayout_3->addWidget(PB_READY);

        PB_START = new QPushButton(verticalLayoutWidget_3);
        PB_START->setObjectName(QStringLiteral("PB_START"));

        verticalLayout_3->addWidget(PB_START);

        PB_STOP = new QPushButton(verticalLayoutWidget_3);
        PB_STOP->setObjectName(QStringLiteral("PB_STOP"));

        verticalLayout_3->addWidget(PB_STOP);

        verticalLayoutWidget_4 = new QWidget(FR_APPROACH);
        verticalLayoutWidget_4->setObjectName(QStringLiteral("verticalLayoutWidget_4"));
        verticalLayoutWidget_4->setGeometry(QRect(140, 20, 191, 125));
        verticalLayout_4 = new QVBoxLayout(verticalLayoutWidget_4);
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        verticalLayout_4->setContentsMargins(0, 0, 0, 10);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(verticalLayoutWidget_4);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        LE_STEP_NUM = new QLineEdit(verticalLayoutWidget_4);
        LE_STEP_NUM->setObjectName(QStringLiteral("LE_STEP_NUM"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LE_STEP_NUM->sizePolicy().hasHeightForWidth());
        LE_STEP_NUM->setSizePolicy(sizePolicy);
        LE_STEP_NUM->setMaximumSize(QSize(100, 16777215));

        horizontalLayout->addWidget(LE_STEP_NUM);


        verticalLayout_4->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(verticalLayoutWidget_4);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        LE_STEP_LENGTH = new QLineEdit(verticalLayoutWidget_4);
        LE_STEP_LENGTH->setObjectName(QStringLiteral("LE_STEP_LENGTH"));
        sizePolicy.setHeightForWidth(LE_STEP_LENGTH->sizePolicy().hasHeightForWidth());
        LE_STEP_LENGTH->setSizePolicy(sizePolicy);
        LE_STEP_LENGTH->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_2->addWidget(LE_STEP_LENGTH);


        verticalLayout_4->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_4 = new QLabel(verticalLayoutWidget_4);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_3->addWidget(label_4);

        LE_STEP_OFFSET = new QLineEdit(verticalLayoutWidget_4);
        LE_STEP_OFFSET->setObjectName(QStringLiteral("LE_STEP_OFFSET"));
        sizePolicy.setHeightForWidth(LE_STEP_OFFSET->sizePolicy().hasHeightForWidth());
        LE_STEP_OFFSET->setSizePolicy(sizePolicy);
        LE_STEP_OFFSET->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_3->addWidget(LE_STEP_OFFSET);


        verticalLayout_4->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_3 = new QLabel(verticalLayoutWidget_4);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_4->addWidget(label_3);

        LE_STEP_ANGLE = new QLineEdit(verticalLayoutWidget_4);
        LE_STEP_ANGLE->setObjectName(QStringLiteral("LE_STEP_ANGLE"));
        sizePolicy.setHeightForWidth(LE_STEP_ANGLE->sizePolicy().hasHeightForWidth());
        LE_STEP_ANGLE->setSizePolicy(sizePolicy);
        LE_STEP_ANGLE->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_4->addWidget(LE_STEP_ANGLE);


        verticalLayout_4->addLayout(horizontalLayout_4);

        MODE_NORMAL = new QRadioButton(FR_APPROACH);
        WALK_MODE = new QButtonGroup(DoorDialog);
        WALK_MODE->setObjectName(QStringLiteral("WALK_MODE"));
        WALK_MODE->addButton(MODE_NORMAL);
        MODE_NORMAL->setObjectName(QStringLiteral("MODE_NORMAL"));
        MODE_NORMAL->setGeometry(QRect(370, 80, 100, 20));
        MODE_LIFTBOX = new QRadioButton(FR_APPROACH);
        WALK_MODE->addButton(MODE_LIFTBOX);
        MODE_LIFTBOX->setObjectName(QStringLiteral("MODE_LIFTBOX"));
        MODE_LIFTBOX->setGeometry(QRect(370, 100, 100, 20));
        MODE_DOOR = new QRadioButton(FR_APPROACH);
        WALK_MODE->addButton(MODE_DOOR);
        MODE_DOOR->setObjectName(QStringLiteral("MODE_DOOR"));
        MODE_DOOR->setGeometry(QRect(370, 120, 100, 20));
        GRP_ZMPFOOT = new QCustomPlot(DoorDialog);
        GRP_ZMPFOOT->setObjectName(QStringLiteral("GRP_ZMPFOOT"));
        GRP_ZMPFOOT->setGeometry(QRect(60, 280, 400, 400));
        CB_GRAPHONOFF = new QCheckBox(DoorDialog);
        CB_GRAPHONOFF->setObjectName(QStringLiteral("CB_GRAPHONOFF"));
        CB_GRAPHONOFF->setGeometry(QRect(470, 290, 85, 20));
        PB_CLEAR_ZMPFOOT = new QPushButton(DoorDialog);
        PB_CLEAR_ZMPFOOT->setObjectName(QStringLiteral("PB_CLEAR_ZMPFOOT"));
        PB_CLEAR_ZMPFOOT->setGeometry(QRect(470, 310, 80, 22));
        widget = new QWidget(DoorDialog);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(520, 0, 251, 111));
        verticalLayout = new QVBoxLayout(widget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        PB_COMPLIANCE_START = new QPushButton(widget);
        PB_COMPLIANCE_START->setObjectName(QStringLiteral("PB_COMPLIANCE_START"));

        verticalLayout->addWidget(PB_COMPLIANCE_START);

        PB_PUSHDOOR = new QPushButton(widget);
        PB_PUSHDOOR->setObjectName(QStringLiteral("PB_PUSHDOOR"));

        verticalLayout->addWidget(PB_PUSHDOOR);

        PB_LEANEDFORWARD = new QPushButton(widget);
        PB_LEANEDFORWARD->setObjectName(QStringLiteral("PB_LEANEDFORWARD"));

        verticalLayout->addWidget(PB_LEANEDFORWARD);

        widget1 = new QWidget(DoorDialog);
        widget1->setObjectName(QStringLiteral("widget1"));
        widget1->setGeometry(QRect(470, 380, 251, 62));
        horizontalLayout_5 = new QHBoxLayout(widget1);
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        horizontalLayout_5->setContentsMargins(0, 0, 0, 0);
        PB_LEANED_B = new QPushButton(widget1);
        PB_LEANED_B->setObjectName(QStringLiteral("PB_LEANED_B"));
        PB_LEANED_B->setMinimumSize(QSize(0, 60));

        horizontalLayout_5->addWidget(PB_LEANED_B);

        PB_LEANED_F = new QPushButton(widget1);
        PB_LEANED_F->setObjectName(QStringLiteral("PB_LEANED_F"));
        PB_LEANED_F->setMinimumSize(QSize(0, 60));

        horizontalLayout_5->addWidget(PB_LEANED_F);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        label_5 = new QLabel(widget1);
        label_5->setObjectName(QStringLiteral("label_5"));

        verticalLayout_2->addWidget(label_5);

        LE_LEANEDF_COMX = new QLineEdit(widget1);
        LE_LEANEDF_COMX->setObjectName(QStringLiteral("LE_LEANEDF_COMX"));

        verticalLayout_2->addWidget(LE_LEANEDF_COMX);


        horizontalLayout_5->addLayout(verticalLayout_2);


        retranslateUi(DoorDialog);

        QMetaObject::connectSlotsByName(DoorDialog);
    } // setupUi

    void retranslateUi(QDialog *DoorDialog)
    {
        DoorDialog->setWindowTitle(QApplication::translate("DoorDialog", "Dialog", 0));
        BTN_DATA_SAVE->setText(QApplication::translate("DoorDialog", "Data Save", 0));
        BTN_SAVE_START->setText(QApplication::translate("DoorDialog", "Save Start", 0));
        BTN_Walk_Ready->setText(QApplication::translate("DoorDialog", "Walk Ready", 0));
        BTN_FORWARD->setText(QApplication::translate("DoorDialog", "Forward", 0));
        BTN_CW->setText(QApplication::translate("DoorDialog", "--CW Right", 0));
        BTN_LEFT->setText(QApplication::translate("DoorDialog", "LEFT", 0));
        BTN_RIGHT->setText(QApplication::translate("DoorDialog", "RIGHT", 0));
        BTN_CCW->setText(QApplication::translate("DoorDialog", "++CCW LEFT", 0));
        PB_READY->setText(QApplication::translate("DoorDialog", "ready", 0));
        PB_START->setText(QApplication::translate("DoorDialog", "start", 0));
        PB_STOP->setText(QApplication::translate("DoorDialog", "stop", 0));
        label->setText(QApplication::translate("DoorDialog", "Step_Num :", 0));
        LE_STEP_NUM->setText(QApplication::translate("DoorDialog", "10", 0));
        label_2->setText(QApplication::translate("DoorDialog", "Step_Len :", 0));
        LE_STEP_LENGTH->setText(QApplication::translate("DoorDialog", "0.001", 0));
        label_4->setText(QApplication::translate("DoorDialog", "Step_Offset :", 0));
        LE_STEP_OFFSET->setText(QApplication::translate("DoorDialog", "0.25", 0));
        label_3->setText(QApplication::translate("DoorDialog", "Step_Ang :", 0));
        LE_STEP_ANGLE->setText(QApplication::translate("DoorDialog", "15", 0));
        MODE_NORMAL->setText(QApplication::translate("DoorDialog", "Normal", 0));
        MODE_LIFTBOX->setText(QApplication::translate("DoorDialog", "LiftBox", 0));
        MODE_DOOR->setText(QApplication::translate("DoorDialog", "Door", 0));
        CB_GRAPHONOFF->setText(QApplication::translate("DoorDialog", "Graph ON", 0));
        PB_CLEAR_ZMPFOOT->setText(QApplication::translate("DoorDialog", "Clear Data", 0));
        PB_COMPLIANCE_START->setText(QApplication::translate("DoorDialog", "Start Compliance Control", 0));
        PB_PUSHDOOR->setText(QApplication::translate("DoorDialog", "Push Door", 0));
        PB_LEANEDFORWARD->setText(QApplication::translate("DoorDialog", "Leaned forward", 0));
        PB_LEANED_B->setText(QApplication::translate("DoorDialog", "BACK", 0));
        PB_LEANED_F->setText(QApplication::translate("DoorDialog", "FRONT", 0));
        label_5->setText(QApplication::translate("DoorDialog", "COMx :", 0));
        LE_LEANEDF_COMX->setText(QApplication::translate("DoorDialog", "0.0", 0));
    } // retranslateUi

};

namespace Ui {
    class DoorDialog: public Ui_DoorDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DOORDIALOG_H
