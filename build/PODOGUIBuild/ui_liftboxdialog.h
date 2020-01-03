/********************************************************************************
** Form generated from reading UI file 'liftboxdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LIFTBOXDIALOG_H
#define UI_LIFTBOXDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_LiftBoxDialog
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
    QWidget *verticalLayoutWidget_5;
    QVBoxLayout *verticalLayout_5;
    QPushButton *PB_SIT_DOWN;
    QPushButton *PB_BOX_;
    QFrame *frame;
    QPushButton *PB_SINGLELOG_WALK;
    QLabel *label_5;
    QWidget *verticalLayoutWidget_6;
    QVBoxLayout *verticalLayout_6;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_6;
    QLineEdit *LE_STEP_NUM_2;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_7;
    QLineEdit *LE_STEP_LENGTH_2;
    QFrame *frame_2;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_7;
    QGridLayout *gridLayout;
    QLabel *label_8;
    QLineEdit *LE_HANDPOS_X;
    QLabel *label_9;
    QLineEdit *LE_HANDPOS_Y;
    QLabel *label_10;
    QLineEdit *LE_HANDPOS_Z;
    QGridLayout *gridLayout_3;
    QLabel *label_15;
    QLineEdit *LE_HANDORI_x;
    QLineEdit *LE_HANDORI_w;
    QLabel *label_16;
    QLabel *label_17;
    QLineEdit *LE_HANDORI_z;
    QLineEdit *LE_HANDORI_y;
    QLabel *label_18;
    QLabel *label_23;
    QLineEdit *LE_HAND_ELB;
    QVBoxLayout *verticalLayout_7;
    QPushButton *PB_RHAND_GO;
    QPushButton *PB_LHAND_GO;
    QWidget *layoutWidget_2;
    QHBoxLayout *horizontalLayout_8;
    QVBoxLayout *verticalLayout_8;
    QRadioButton *RB_HANDboth;
    QRadioButton *RB_HANDr;
    QRadioButton *RB_HANDl;
    QPushButton *PB_HAND_GRASP;
    QPushButton *PB_HAND_STOP;
    QPushButton *PB_HAND_OPEN;
    QLabel *label_14;
    QTabWidget *tabWidget;
    QWidget *TAB_CVRHAND;
    QPushButton *PB_RCV_Reset;
    QPushButton *PB_RCV_Push;
    QTableWidget *tableWidget;
    QWidget *TAB_CVLHAND;
    QPushButton *PB_LCV_Push;
    QPushButton *PB_LCV_Reset;
    QTableWidget *tableWidgetL;
    QWidget *tab;
    QTableWidget *tableWidgetSave;
    QPushButton *PB_SAVE_Push;
    QPushButton *PB_SAVE_Reset;
    QWidget *layoutWidget_4;
    QHBoxLayout *horizontalLayout_9;
    QGridLayout *gridLayout_2;
    QLabel *label_11;
    QLineEdit *LE_FKPOS_X;
    QLabel *label_12;
    QLineEdit *LE_FKPOS_Y;
    QLabel *label_13;
    QLineEdit *LE_FKPOS_Z;
    QGridLayout *gridLayout_4;
    QLabel *label_19;
    QLineEdit *LE_FKORI_x;
    QLineEdit *LE_FKORI_w;
    QLabel *label_20;
    QLabel *label_21;
    QLineEdit *LE_FKORI_z;
    QLineEdit *LE_FKORI_y;
    QLabel *label_22;
    QLabel *label_24;
    QLineEdit *LE_FKELB;
    QPushButton *PB_GOR;
    QPushButton *PB_GOL;
    QPushButton *PB_LOCK;
    QPushButton *PB_SAVE_save;
    QPushButton *pushButton;
    QPushButton *PB_GOL_Test;
    QPushButton *PB_CHANGE_INTERPOLATION;
    QLineEdit *LE_INTERPOLATION_DEG;
    QLineEdit *LE_INTERPOLATION_T;
    QPushButton *PB_STEPPING_STONE;
    QWidget *verticalLayoutWidget;
    QVBoxLayout *verticalLayout;
    QPushButton *PB_LIFTBOX_BACK;
    QPushButton *PB_LIFTBOX_SIT;
    QPushButton *PB_LIFTBOX_HOLD;
    QPushButton *PB_LIFTBOX_STAND;
    QPushButton *PB_LIFTBOX_FRONT;

    void setupUi(QDialog *LiftBoxDialog)
    {
        if (LiftBoxDialog->objectName().isEmpty())
            LiftBoxDialog->setObjectName(QStringLiteral("LiftBoxDialog"));
        LiftBoxDialog->resize(760, 729);
        QSizePolicy sizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(LiftBoxDialog->sizePolicy().hasHeightForWidth());
        LiftBoxDialog->setSizePolicy(sizePolicy);
        LiftBoxDialog->setMinimumSize(QSize(0, 50));
        FR_APPROACH = new QFrame(LiftBoxDialog);
        FR_APPROACH->setObjectName(QStringLiteral("FR_APPROACH"));
        FR_APPROACH->setGeometry(QRect(10, 10, 491, 271));
        FR_APPROACH->setFrameShape(QFrame::StyledPanel);
        FR_APPROACH->setFrameShadow(QFrame::Raised);
        BTN_DATA_SAVE = new QPushButton(FR_APPROACH);
        BTN_DATA_SAVE->setObjectName(QStringLiteral("BTN_DATA_SAVE"));
        BTN_DATA_SAVE->setGeometry(QRect(130, 200, 81, 31));
        BTN_SAVE_START = new QPushButton(FR_APPROACH);
        BTN_SAVE_START->setObjectName(QStringLiteral("BTN_SAVE_START"));
        BTN_SAVE_START->setGeometry(QRect(130, 164, 81, 31));
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
        verticalLayoutWidget_3->setGeometry(QRect(230, 160, 101, 80));
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
        verticalLayout_4->setContentsMargins(0, 0, 0, 0);
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(verticalLayoutWidget_4);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        LE_STEP_NUM = new QLineEdit(verticalLayoutWidget_4);
        LE_STEP_NUM->setObjectName(QStringLiteral("LE_STEP_NUM"));
        QSizePolicy sizePolicy1(QSizePolicy::Fixed, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(LE_STEP_NUM->sizePolicy().hasHeightForWidth());
        LE_STEP_NUM->setSizePolicy(sizePolicy1);
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
        sizePolicy1.setHeightForWidth(LE_STEP_LENGTH->sizePolicy().hasHeightForWidth());
        LE_STEP_LENGTH->setSizePolicy(sizePolicy1);
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
        sizePolicy1.setHeightForWidth(LE_STEP_OFFSET->sizePolicy().hasHeightForWidth());
        LE_STEP_OFFSET->setSizePolicy(sizePolicy1);
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
        sizePolicy1.setHeightForWidth(LE_STEP_ANGLE->sizePolicy().hasHeightForWidth());
        LE_STEP_ANGLE->setSizePolicy(sizePolicy1);
        LE_STEP_ANGLE->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_4->addWidget(LE_STEP_ANGLE);


        verticalLayout_4->addLayout(horizontalLayout_4);

        verticalLayoutWidget_5 = new QWidget(FR_APPROACH);
        verticalLayoutWidget_5->setObjectName(QStringLiteral("verticalLayoutWidget_5"));
        verticalLayoutWidget_5->setGeometry(QRect(360, 70, 121, 136));
        verticalLayout_5 = new QVBoxLayout(verticalLayoutWidget_5);
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        verticalLayout_5->setContentsMargins(0, 0, 0, 0);
        PB_SIT_DOWN = new QPushButton(verticalLayoutWidget_5);
        PB_SIT_DOWN->setObjectName(QStringLiteral("PB_SIT_DOWN"));

        verticalLayout_5->addWidget(PB_SIT_DOWN);

        PB_BOX_ = new QPushButton(verticalLayoutWidget_5);
        PB_BOX_->setObjectName(QStringLiteral("PB_BOX_"));

        verticalLayout_5->addWidget(PB_BOX_);

        frame = new QFrame(LiftBoxDialog);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(510, 10, 241, 271));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        PB_SINGLELOG_WALK = new QPushButton(frame);
        PB_SINGLELOG_WALK->setObjectName(QStringLiteral("PB_SINGLELOG_WALK"));
        PB_SINGLELOG_WALK->setGeometry(QRect(20, 50, 201, 61));
        label_5 = new QLabel(frame);
        label_5->setObjectName(QStringLiteral("label_5"));
        label_5->setGeometry(QRect(10, 10, 221, 31));
        verticalLayoutWidget_6 = new QWidget(frame);
        verticalLayoutWidget_6->setObjectName(QStringLiteral("verticalLayoutWidget_6"));
        verticalLayoutWidget_6->setGeometry(QRect(20, 130, 191, 81));
        verticalLayout_6 = new QVBoxLayout(verticalLayoutWidget_6);
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        verticalLayout_6->setContentsMargins(0, 0, 0, 0);
        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_6 = new QLabel(verticalLayoutWidget_6);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_5->addWidget(label_6);

        LE_STEP_NUM_2 = new QLineEdit(verticalLayoutWidget_6);
        LE_STEP_NUM_2->setObjectName(QStringLiteral("LE_STEP_NUM_2"));
        sizePolicy1.setHeightForWidth(LE_STEP_NUM_2->sizePolicy().hasHeightForWidth());
        LE_STEP_NUM_2->setSizePolicy(sizePolicy1);
        LE_STEP_NUM_2->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_5->addWidget(LE_STEP_NUM_2);


        verticalLayout_6->addLayout(horizontalLayout_5);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_7 = new QLabel(verticalLayoutWidget_6);
        label_7->setObjectName(QStringLiteral("label_7"));

        horizontalLayout_6->addWidget(label_7);

        LE_STEP_LENGTH_2 = new QLineEdit(verticalLayoutWidget_6);
        LE_STEP_LENGTH_2->setObjectName(QStringLiteral("LE_STEP_LENGTH_2"));
        sizePolicy1.setHeightForWidth(LE_STEP_LENGTH_2->sizePolicy().hasHeightForWidth());
        LE_STEP_LENGTH_2->setSizePolicy(sizePolicy1);
        LE_STEP_LENGTH_2->setMaximumSize(QSize(100, 16777215));

        horizontalLayout_6->addWidget(LE_STEP_LENGTH_2);


        verticalLayout_6->addLayout(horizontalLayout_6);

        label_5->raise();
        verticalLayoutWidget_6->raise();
        PB_SINGLELOG_WALK->raise();
        frame_2 = new QFrame(LiftBoxDialog);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setGeometry(QRect(400, 690, 351, 411));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        layoutWidget = new QWidget(frame_2);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 60, 331, 138));
        horizontalLayout_7 = new QHBoxLayout(layoutWidget);
        horizontalLayout_7->setObjectName(QStringLiteral("horizontalLayout_7"));
        horizontalLayout_7->setContentsMargins(0, 0, 0, 0);
        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QStringLiteral("label_8"));

        gridLayout->addWidget(label_8, 0, 0, 1, 1);

        LE_HANDPOS_X = new QLineEdit(layoutWidget);
        LE_HANDPOS_X->setObjectName(QStringLiteral("LE_HANDPOS_X"));

        gridLayout->addWidget(LE_HANDPOS_X, 0, 1, 1, 1);

        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QStringLiteral("label_9"));

        gridLayout->addWidget(label_9, 1, 0, 1, 1);

        LE_HANDPOS_Y = new QLineEdit(layoutWidget);
        LE_HANDPOS_Y->setObjectName(QStringLiteral("LE_HANDPOS_Y"));

        gridLayout->addWidget(LE_HANDPOS_Y, 1, 1, 1, 1);

        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QStringLiteral("label_10"));

        gridLayout->addWidget(label_10, 2, 0, 1, 1);

        LE_HANDPOS_Z = new QLineEdit(layoutWidget);
        LE_HANDPOS_Z->setObjectName(QStringLiteral("LE_HANDPOS_Z"));

        gridLayout->addWidget(LE_HANDPOS_Z, 2, 1, 1, 1);


        horizontalLayout_7->addLayout(gridLayout);

        gridLayout_3 = new QGridLayout();
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        label_15 = new QLabel(layoutWidget);
        label_15->setObjectName(QStringLiteral("label_15"));

        gridLayout_3->addWidget(label_15, 2, 0, 1, 1);

        LE_HANDORI_x = new QLineEdit(layoutWidget);
        LE_HANDORI_x->setObjectName(QStringLiteral("LE_HANDORI_x"));

        gridLayout_3->addWidget(LE_HANDORI_x, 1, 1, 1, 1);

        LE_HANDORI_w = new QLineEdit(layoutWidget);
        LE_HANDORI_w->setObjectName(QStringLiteral("LE_HANDORI_w"));

        gridLayout_3->addWidget(LE_HANDORI_w, 0, 1, 1, 1);

        label_16 = new QLabel(layoutWidget);
        label_16->setObjectName(QStringLiteral("label_16"));

        gridLayout_3->addWidget(label_16, 1, 0, 1, 1);

        label_17 = new QLabel(layoutWidget);
        label_17->setObjectName(QStringLiteral("label_17"));

        gridLayout_3->addWidget(label_17, 0, 0, 1, 1);

        LE_HANDORI_z = new QLineEdit(layoutWidget);
        LE_HANDORI_z->setObjectName(QStringLiteral("LE_HANDORI_z"));

        gridLayout_3->addWidget(LE_HANDORI_z, 3, 1, 1, 1);

        LE_HANDORI_y = new QLineEdit(layoutWidget);
        LE_HANDORI_y->setObjectName(QStringLiteral("LE_HANDORI_y"));

        gridLayout_3->addWidget(LE_HANDORI_y, 2, 1, 1, 1);

        label_18 = new QLabel(layoutWidget);
        label_18->setObjectName(QStringLiteral("label_18"));

        gridLayout_3->addWidget(label_18, 3, 0, 1, 1);

        label_23 = new QLabel(layoutWidget);
        label_23->setObjectName(QStringLiteral("label_23"));

        gridLayout_3->addWidget(label_23, 4, 0, 1, 1);

        LE_HAND_ELB = new QLineEdit(layoutWidget);
        LE_HAND_ELB->setObjectName(QStringLiteral("LE_HAND_ELB"));

        gridLayout_3->addWidget(LE_HAND_ELB, 4, 1, 1, 1);


        horizontalLayout_7->addLayout(gridLayout_3);

        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        PB_RHAND_GO = new QPushButton(layoutWidget);
        PB_RHAND_GO->setObjectName(QStringLiteral("PB_RHAND_GO"));
        PB_RHAND_GO->setMinimumSize(QSize(0, 40));

        verticalLayout_7->addWidget(PB_RHAND_GO);

        PB_LHAND_GO = new QPushButton(layoutWidget);
        PB_LHAND_GO->setObjectName(QStringLiteral("PB_LHAND_GO"));
        PB_LHAND_GO->setMinimumSize(QSize(0, 40));

        verticalLayout_7->addWidget(PB_LHAND_GO);


        horizontalLayout_7->addLayout(verticalLayout_7);

        layoutWidget_2 = new QWidget(frame_2);
        layoutWidget_2->setObjectName(QStringLiteral("layoutWidget_2"));
        layoutWidget_2->setGeometry(QRect(10, 240, 321, 76));
        horizontalLayout_8 = new QHBoxLayout(layoutWidget_2);
        horizontalLayout_8->setObjectName(QStringLiteral("horizontalLayout_8"));
        horizontalLayout_8->setContentsMargins(0, 0, 0, 0);
        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        RB_HANDboth = new QRadioButton(layoutWidget_2);
        RB_HANDboth->setObjectName(QStringLiteral("RB_HANDboth"));

        verticalLayout_8->addWidget(RB_HANDboth);

        RB_HANDr = new QRadioButton(layoutWidget_2);
        RB_HANDr->setObjectName(QStringLiteral("RB_HANDr"));

        verticalLayout_8->addWidget(RB_HANDr);

        RB_HANDl = new QRadioButton(layoutWidget_2);
        RB_HANDl->setObjectName(QStringLiteral("RB_HANDl"));

        verticalLayout_8->addWidget(RB_HANDl);


        horizontalLayout_8->addLayout(verticalLayout_8);

        PB_HAND_GRASP = new QPushButton(layoutWidget_2);
        PB_HAND_GRASP->setObjectName(QStringLiteral("PB_HAND_GRASP"));
        PB_HAND_GRASP->setMinimumSize(QSize(30, 40));

        horizontalLayout_8->addWidget(PB_HAND_GRASP);

        PB_HAND_STOP = new QPushButton(layoutWidget_2);
        PB_HAND_STOP->setObjectName(QStringLiteral("PB_HAND_STOP"));
        PB_HAND_STOP->setMinimumSize(QSize(30, 40));

        horizontalLayout_8->addWidget(PB_HAND_STOP);

        PB_HAND_OPEN = new QPushButton(layoutWidget_2);
        PB_HAND_OPEN->setObjectName(QStringLiteral("PB_HAND_OPEN"));
        PB_HAND_OPEN->setMinimumSize(QSize(30, 40));

        horizontalLayout_8->addWidget(PB_HAND_OPEN);

        label_14 = new QLabel(frame_2);
        label_14->setObjectName(QStringLiteral("label_14"));
        label_14->setGeometry(QRect(10, 0, 201, 41));
        tabWidget = new QTabWidget(LiftBoxDialog);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(360, 290, 381, 411));
        TAB_CVRHAND = new QWidget();
        TAB_CVRHAND->setObjectName(QStringLiteral("TAB_CVRHAND"));
        PB_RCV_Reset = new QPushButton(TAB_CVRHAND);
        PB_RCV_Reset->setObjectName(QStringLiteral("PB_RCV_Reset"));
        PB_RCV_Reset->setGeometry(QRect(202, 10, 80, 22));
        PB_RCV_Push = new QPushButton(TAB_CVRHAND);
        PB_RCV_Push->setObjectName(QStringLiteral("PB_RCV_Push"));
        PB_RCV_Push->setGeometry(QRect(290, 10, 80, 22));
        tableWidget = new QTableWidget(TAB_CVRHAND);
        tableWidget->setObjectName(QStringLiteral("tableWidget"));
        tableWidget->setGeometry(QRect(3, 50, 371, 331));
        tabWidget->addTab(TAB_CVRHAND, QString());
        TAB_CVLHAND = new QWidget();
        TAB_CVLHAND->setObjectName(QStringLiteral("TAB_CVLHAND"));
        PB_LCV_Push = new QPushButton(TAB_CVLHAND);
        PB_LCV_Push->setObjectName(QStringLiteral("PB_LCV_Push"));
        PB_LCV_Push->setGeometry(QRect(290, 10, 80, 22));
        PB_LCV_Reset = new QPushButton(TAB_CVLHAND);
        PB_LCV_Reset->setObjectName(QStringLiteral("PB_LCV_Reset"));
        PB_LCV_Reset->setGeometry(QRect(202, 10, 80, 22));
        tableWidgetL = new QTableWidget(TAB_CVLHAND);
        tableWidgetL->setObjectName(QStringLiteral("tableWidgetL"));
        tableWidgetL->setGeometry(QRect(3, 50, 371, 331));
        tabWidget->addTab(TAB_CVLHAND, QString());
        tab = new QWidget();
        tab->setObjectName(QStringLiteral("tab"));
        tableWidgetSave = new QTableWidget(tab);
        tableWidgetSave->setObjectName(QStringLiteral("tableWidgetSave"));
        tableWidgetSave->setGeometry(QRect(3, 190, 371, 191));
        PB_SAVE_Push = new QPushButton(tab);
        PB_SAVE_Push->setObjectName(QStringLiteral("PB_SAVE_Push"));
        PB_SAVE_Push->setGeometry(QRect(290, 170, 80, 22));
        PB_SAVE_Reset = new QPushButton(tab);
        PB_SAVE_Reset->setObjectName(QStringLiteral("PB_SAVE_Reset"));
        PB_SAVE_Reset->setGeometry(QRect(200, 170, 80, 22));
        layoutWidget_4 = new QWidget(tab);
        layoutWidget_4->setObjectName(QStringLiteral("layoutWidget_4"));
        layoutWidget_4->setGeometry(QRect(130, 10, 241, 138));
        horizontalLayout_9 = new QHBoxLayout(layoutWidget_4);
        horizontalLayout_9->setObjectName(QStringLiteral("horizontalLayout_9"));
        horizontalLayout_9->setContentsMargins(0, 0, 0, 0);
        gridLayout_2 = new QGridLayout();
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        label_11 = new QLabel(layoutWidget_4);
        label_11->setObjectName(QStringLiteral("label_11"));

        gridLayout_2->addWidget(label_11, 0, 0, 1, 1);

        LE_FKPOS_X = new QLineEdit(layoutWidget_4);
        LE_FKPOS_X->setObjectName(QStringLiteral("LE_FKPOS_X"));

        gridLayout_2->addWidget(LE_FKPOS_X, 0, 1, 1, 1);

        label_12 = new QLabel(layoutWidget_4);
        label_12->setObjectName(QStringLiteral("label_12"));

        gridLayout_2->addWidget(label_12, 1, 0, 1, 1);

        LE_FKPOS_Y = new QLineEdit(layoutWidget_4);
        LE_FKPOS_Y->setObjectName(QStringLiteral("LE_FKPOS_Y"));

        gridLayout_2->addWidget(LE_FKPOS_Y, 1, 1, 1, 1);

        label_13 = new QLabel(layoutWidget_4);
        label_13->setObjectName(QStringLiteral("label_13"));

        gridLayout_2->addWidget(label_13, 2, 0, 1, 1);

        LE_FKPOS_Z = new QLineEdit(layoutWidget_4);
        LE_FKPOS_Z->setObjectName(QStringLiteral("LE_FKPOS_Z"));

        gridLayout_2->addWidget(LE_FKPOS_Z, 2, 1, 1, 1);


        horizontalLayout_9->addLayout(gridLayout_2);

        gridLayout_4 = new QGridLayout();
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        label_19 = new QLabel(layoutWidget_4);
        label_19->setObjectName(QStringLiteral("label_19"));

        gridLayout_4->addWidget(label_19, 2, 0, 1, 1);

        LE_FKORI_x = new QLineEdit(layoutWidget_4);
        LE_FKORI_x->setObjectName(QStringLiteral("LE_FKORI_x"));

        gridLayout_4->addWidget(LE_FKORI_x, 1, 1, 1, 1);

        LE_FKORI_w = new QLineEdit(layoutWidget_4);
        LE_FKORI_w->setObjectName(QStringLiteral("LE_FKORI_w"));

        gridLayout_4->addWidget(LE_FKORI_w, 0, 1, 1, 1);

        label_20 = new QLabel(layoutWidget_4);
        label_20->setObjectName(QStringLiteral("label_20"));

        gridLayout_4->addWidget(label_20, 1, 0, 1, 1);

        label_21 = new QLabel(layoutWidget_4);
        label_21->setObjectName(QStringLiteral("label_21"));

        gridLayout_4->addWidget(label_21, 0, 0, 1, 1);

        LE_FKORI_z = new QLineEdit(layoutWidget_4);
        LE_FKORI_z->setObjectName(QStringLiteral("LE_FKORI_z"));

        gridLayout_4->addWidget(LE_FKORI_z, 3, 1, 1, 1);

        LE_FKORI_y = new QLineEdit(layoutWidget_4);
        LE_FKORI_y->setObjectName(QStringLiteral("LE_FKORI_y"));

        gridLayout_4->addWidget(LE_FKORI_y, 2, 1, 1, 1);

        label_22 = new QLabel(layoutWidget_4);
        label_22->setObjectName(QStringLiteral("label_22"));

        gridLayout_4->addWidget(label_22, 3, 0, 1, 1);

        label_24 = new QLabel(layoutWidget_4);
        label_24->setObjectName(QStringLiteral("label_24"));

        gridLayout_4->addWidget(label_24, 4, 0, 1, 1);

        LE_FKELB = new QLineEdit(layoutWidget_4);
        LE_FKELB->setObjectName(QStringLiteral("LE_FKELB"));

        gridLayout_4->addWidget(LE_FKELB, 4, 1, 1, 1);


        horizontalLayout_9->addLayout(gridLayout_4);

        PB_GOR = new QPushButton(tab);
        PB_GOR->setObjectName(QStringLiteral("PB_GOR"));
        PB_GOR->setGeometry(QRect(10, 25, 111, 31));
        PB_GOL = new QPushButton(tab);
        PB_GOL->setObjectName(QStringLiteral("PB_GOL"));
        PB_GOL->setGeometry(QRect(10, 65, 111, 31));
        PB_LOCK = new QPushButton(tab);
        PB_LOCK->setObjectName(QStringLiteral("PB_LOCK"));
        PB_LOCK->setGeometry(QRect(10, 105, 111, 31));
        PB_SAVE_save = new QPushButton(tab);
        PB_SAVE_save->setObjectName(QStringLiteral("PB_SAVE_save"));
        PB_SAVE_save->setGeometry(QRect(110, 170, 80, 22));
        pushButton = new QPushButton(tab);
        pushButton->setObjectName(QStringLiteral("pushButton"));
        pushButton->setGeometry(QRect(10, 170, 80, 22));
        PB_GOL_Test = new QPushButton(tab);
        PB_GOL_Test->setObjectName(QStringLiteral("PB_GOL_Test"));
        PB_GOL_Test->setGeometry(QRect(10, 140, 111, 21));
        tabWidget->addTab(tab, QString());
        PB_CHANGE_INTERPOLATION = new QPushButton(LiftBoxDialog);
        PB_CHANGE_INTERPOLATION->setObjectName(QStringLiteral("PB_CHANGE_INTERPOLATION"));
        PB_CHANGE_INTERPOLATION->setGeometry(QRect(10, 360, 80, 51));
        LE_INTERPOLATION_DEG = new QLineEdit(LiftBoxDialog);
        LE_INTERPOLATION_DEG->setObjectName(QStringLiteral("LE_INTERPOLATION_DEG"));
        LE_INTERPOLATION_DEG->setGeometry(QRect(10, 290, 113, 22));
        LE_INTERPOLATION_T = new QLineEdit(LiftBoxDialog);
        LE_INTERPOLATION_T->setObjectName(QStringLiteral("LE_INTERPOLATION_T"));
        LE_INTERPOLATION_T->setGeometry(QRect(10, 320, 113, 22));
        PB_STEPPING_STONE = new QPushButton(LiftBoxDialog);
        PB_STEPPING_STONE->setObjectName(QStringLiteral("PB_STEPPING_STONE"));
        PB_STEPPING_STONE->setGeometry(QRect(30, 440, 121, 51));
        verticalLayoutWidget = new QWidget(LiftBoxDialog);
        verticalLayoutWidget->setObjectName(QStringLiteral("verticalLayoutWidget"));
        verticalLayoutWidget->setGeometry(QRect(170, 330, 160, 136));
        verticalLayout = new QVBoxLayout(verticalLayoutWidget);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        verticalLayout->setContentsMargins(0, 0, 0, 0);
        PB_LIFTBOX_BACK = new QPushButton(verticalLayoutWidget);
        PB_LIFTBOX_BACK->setObjectName(QStringLiteral("PB_LIFTBOX_BACK"));

        verticalLayout->addWidget(PB_LIFTBOX_BACK);

        PB_LIFTBOX_SIT = new QPushButton(verticalLayoutWidget);
        PB_LIFTBOX_SIT->setObjectName(QStringLiteral("PB_LIFTBOX_SIT"));

        verticalLayout->addWidget(PB_LIFTBOX_SIT);

        PB_LIFTBOX_HOLD = new QPushButton(verticalLayoutWidget);
        PB_LIFTBOX_HOLD->setObjectName(QStringLiteral("PB_LIFTBOX_HOLD"));

        verticalLayout->addWidget(PB_LIFTBOX_HOLD);

        PB_LIFTBOX_STAND = new QPushButton(verticalLayoutWidget);
        PB_LIFTBOX_STAND->setObjectName(QStringLiteral("PB_LIFTBOX_STAND"));

        verticalLayout->addWidget(PB_LIFTBOX_STAND);

        PB_LIFTBOX_FRONT = new QPushButton(verticalLayoutWidget);
        PB_LIFTBOX_FRONT->setObjectName(QStringLiteral("PB_LIFTBOX_FRONT"));

        verticalLayout->addWidget(PB_LIFTBOX_FRONT);

        tabWidget->raise();
        FR_APPROACH->raise();
        frame->raise();
        frame_2->raise();
        PB_CHANGE_INTERPOLATION->raise();
        LE_INTERPOLATION_DEG->raise();
        LE_INTERPOLATION_T->raise();
        PB_STEPPING_STONE->raise();
        verticalLayoutWidget->raise();

        retranslateUi(LiftBoxDialog);

        tabWidget->setCurrentIndex(2);


        QMetaObject::connectSlotsByName(LiftBoxDialog);
    } // setupUi

    void retranslateUi(QDialog *LiftBoxDialog)
    {
        LiftBoxDialog->setWindowTitle(QApplication::translate("LiftBoxDialog", "Dialog", 0));
        BTN_DATA_SAVE->setText(QApplication::translate("LiftBoxDialog", "Data Save", 0));
        BTN_SAVE_START->setText(QApplication::translate("LiftBoxDialog", "Save Start", 0));
        BTN_Walk_Ready->setText(QApplication::translate("LiftBoxDialog", "Walk Ready", 0));
        BTN_FORWARD->setText(QApplication::translate("LiftBoxDialog", "Forward", 0));
        BTN_CW->setText(QApplication::translate("LiftBoxDialog", "--CW Right", 0));
        BTN_LEFT->setText(QApplication::translate("LiftBoxDialog", "LEFT", 0));
        BTN_RIGHT->setText(QApplication::translate("LiftBoxDialog", "RIGHT", 0));
        BTN_CCW->setText(QApplication::translate("LiftBoxDialog", "++CCW LEFT", 0));
        PB_READY->setText(QApplication::translate("LiftBoxDialog", "ready", 0));
        PB_START->setText(QApplication::translate("LiftBoxDialog", "start", 0));
        PB_STOP->setText(QApplication::translate("LiftBoxDialog", "stop", 0));
        label->setText(QApplication::translate("LiftBoxDialog", "Step_Num :", 0));
        LE_STEP_NUM->setText(QApplication::translate("LiftBoxDialog", "10", 0));
        label_2->setText(QApplication::translate("LiftBoxDialog", "Step_Len :", 0));
        LE_STEP_LENGTH->setText(QApplication::translate("LiftBoxDialog", "0.001", 0));
        label_4->setText(QApplication::translate("LiftBoxDialog", "Step_Offset :", 0));
        LE_STEP_OFFSET->setText(QApplication::translate("LiftBoxDialog", "0.25", 0));
        label_3->setText(QApplication::translate("LiftBoxDialog", "Step_Ang :", 0));
        LE_STEP_ANGLE->setText(QApplication::translate("LiftBoxDialog", "15", 0));
        PB_SIT_DOWN->setText(QApplication::translate("LiftBoxDialog", "LiftBox", 0));
        PB_BOX_->setText(QApplication::translate("LiftBoxDialog", "PutBox", 0));
        PB_SINGLELOG_WALK->setText(QApplication::translate("LiftBoxDialog", "Walking Start", 0));
        label_5->setText(QApplication::translate("LiftBoxDialog", "<html><head/><body><p><span style=\" font-size:14pt; font-weight:600;\">Single Log Walking</span></p></body></html>", 0));
        label_6->setText(QApplication::translate("LiftBoxDialog", "Step_Num :", 0));
        LE_STEP_NUM_2->setText(QApplication::translate("LiftBoxDialog", "10", 0));
        label_7->setText(QApplication::translate("LiftBoxDialog", "Step_Len :", 0));
        LE_STEP_LENGTH_2->setText(QApplication::translate("LiftBoxDialog", "0.001", 0));
        label_8->setText(QApplication::translate("LiftBoxDialog", "X :", 0));
        label_9->setText(QApplication::translate("LiftBoxDialog", "Y :", 0));
        label_10->setText(QApplication::translate("LiftBoxDialog", "Z :", 0));
        label_15->setText(QApplication::translate("LiftBoxDialog", "y :", 0));
        label_16->setText(QApplication::translate("LiftBoxDialog", "x :", 0));
        label_17->setText(QApplication::translate("LiftBoxDialog", "w :", 0));
        label_18->setText(QApplication::translate("LiftBoxDialog", "z :", 0));
        label_23->setText(QApplication::translate("LiftBoxDialog", "Elb:", 0));
        PB_RHAND_GO->setText(QApplication::translate("LiftBoxDialog", "RHAND GO", 0));
        PB_LHAND_GO->setText(QApplication::translate("LiftBoxDialog", "LHAND GO", 0));
        RB_HANDboth->setText(QApplication::translate("LiftBoxDialog", "Both", 0));
        RB_HANDr->setText(QApplication::translate("LiftBoxDialog", "RHAND only", 0));
        RB_HANDl->setText(QApplication::translate("LiftBoxDialog", "LHAND only", 0));
        PB_HAND_GRASP->setText(QApplication::translate("LiftBoxDialog", "Grasp", 0));
        PB_HAND_STOP->setText(QApplication::translate("LiftBoxDialog", "Stop", 0));
        PB_HAND_OPEN->setText(QApplication::translate("LiftBoxDialog", "Open", 0));
        label_14->setText(QApplication::translate("LiftBoxDialog", "<html><head/><body><p><span style=\" font-size:18pt; font-weight:600;\">Grasping test</span></p></body></html>", 0));
        PB_RCV_Reset->setText(QApplication::translate("LiftBoxDialog", "Reset", 0));
        PB_RCV_Push->setText(QApplication::translate("LiftBoxDialog", "Push ->", 0));
        tabWidget->setTabText(tabWidget->indexOf(TAB_CVRHAND), QApplication::translate("LiftBoxDialog", "RHAND", 0));
        PB_LCV_Push->setText(QApplication::translate("LiftBoxDialog", "Push ->", 0));
        PB_LCV_Reset->setText(QApplication::translate("LiftBoxDialog", "Reset", 0));
        tabWidget->setTabText(tabWidget->indexOf(TAB_CVLHAND), QApplication::translate("LiftBoxDialog", "LHAND", 0));
        PB_SAVE_Push->setText(QApplication::translate("LiftBoxDialog", "Push ->", 0));
        PB_SAVE_Reset->setText(QApplication::translate("LiftBoxDialog", "Reset", 0));
        label_11->setText(QApplication::translate("LiftBoxDialog", "X :", 0));
        label_12->setText(QApplication::translate("LiftBoxDialog", "Y :", 0));
        label_13->setText(QApplication::translate("LiftBoxDialog", "Z :", 0));
        label_19->setText(QApplication::translate("LiftBoxDialog", "y :", 0));
        label_20->setText(QApplication::translate("LiftBoxDialog", "x :", 0));
        label_21->setText(QApplication::translate("LiftBoxDialog", "w :", 0));
        label_22->setText(QApplication::translate("LiftBoxDialog", "z :", 0));
        label_24->setText(QApplication::translate("LiftBoxDialog", "Elb:", 0));
        PB_GOR->setText(QApplication::translate("LiftBoxDialog", "GainOverrideR", 0));
        PB_GOL->setText(QApplication::translate("LiftBoxDialog", "GainOverrideL", 0));
        PB_LOCK->setText(QApplication::translate("LiftBoxDialog", "Lock Arm", 0));
        PB_SAVE_save->setText(QApplication::translate("LiftBoxDialog", "Save", 0));
        pushButton->setText(QApplication::translate("LiftBoxDialog", ".txt", 0));
        PB_GOL_Test->setText(QApplication::translate("LiftBoxDialog", "GO R test", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab), QApplication::translate("LiftBoxDialog", "Save", 0));
        PB_CHANGE_INTERPOLATION->setText(QApplication::translate("LiftBoxDialog", "ChangePos", 0));
        LE_INTERPOLATION_DEG->setText(QApplication::translate("LiftBoxDialog", "0.0", 0));
        LE_INTERPOLATION_T->setText(QApplication::translate("LiftBoxDialog", "0.0", 0));
        PB_STEPPING_STONE->setText(QApplication::translate("LiftBoxDialog", "Stepping Stone", 0));
        PB_LIFTBOX_BACK->setText(QApplication::translate("LiftBoxDialog", "Back", 0));
        PB_LIFTBOX_SIT->setText(QApplication::translate("LiftBoxDialog", "SitDown", 0));
        PB_LIFTBOX_HOLD->setText(QApplication::translate("LiftBoxDialog", "HoldBox", 0));
        PB_LIFTBOX_STAND->setText(QApplication::translate("LiftBoxDialog", "Standup", 0));
        PB_LIFTBOX_FRONT->setText(QApplication::translate("LiftBoxDialog", "Front", 0));
    } // retranslateUi

};

namespace Ui {
    class LiftBoxDialog: public Ui_LiftBoxDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIFTBOXDIALOG_H
