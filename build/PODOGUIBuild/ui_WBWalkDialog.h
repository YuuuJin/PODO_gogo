/********************************************************************************
** Form generated from reading UI file 'WBWalkDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_WBWALKDIALOG_H
#define UI_WBWALKDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_WBWalkDialog
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
    QFrame *frame;
    QPushButton *PB_APPROACH_H;
    QPushButton *PB_GRASP_H;
    QPushButton *PB_QUIT_H;

    void setupUi(QDialog *WBWalkDialog)
    {
        if (WBWalkDialog->objectName().isEmpty())
            WBWalkDialog->setObjectName(QStringLiteral("WBWalkDialog"));
        WBWalkDialog->resize(825, 707);
        FR_APPROACH = new QFrame(WBWalkDialog);
        FR_APPROACH->setObjectName(QStringLiteral("FR_APPROACH"));
        FR_APPROACH->setGeometry(QRect(10, 30, 351, 271));
        FR_APPROACH->setFrameShape(QFrame::StyledPanel);
        FR_APPROACH->setFrameShadow(QFrame::Raised);
        BTN_DATA_SAVE = new QPushButton(FR_APPROACH);
        BTN_DATA_SAVE->setObjectName(QStringLiteral("BTN_DATA_SAVE"));
        BTN_DATA_SAVE->setGeometry(QRect(140, 200, 81, 31));
        BTN_SAVE_START = new QPushButton(FR_APPROACH);
        BTN_SAVE_START->setObjectName(QStringLiteral("BTN_SAVE_START"));
        BTN_SAVE_START->setGeometry(QRect(140, 160, 81, 31));
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

        frame = new QFrame(WBWalkDialog);
        frame->setObjectName(QStringLiteral("frame"));
        frame->setGeometry(QRect(10, 310, 351, 291));
        frame->setFrameShape(QFrame::StyledPanel);
        frame->setFrameShadow(QFrame::Raised);
        PB_APPROACH_H = new QPushButton(frame);
        PB_APPROACH_H->setObjectName(QStringLiteral("PB_APPROACH_H"));
        PB_APPROACH_H->setGeometry(QRect(80, 20, 181, 51));
        PB_GRASP_H = new QPushButton(frame);
        PB_GRASP_H->setObjectName(QStringLiteral("PB_GRASP_H"));
        PB_GRASP_H->setGeometry(QRect(80, 80, 181, 51));
        PB_QUIT_H = new QPushButton(frame);
        PB_QUIT_H->setObjectName(QStringLiteral("PB_QUIT_H"));
        PB_QUIT_H->setGeometry(QRect(80, 140, 181, 51));

        retranslateUi(WBWalkDialog);

        QMetaObject::connectSlotsByName(WBWalkDialog);
    } // setupUi

    void retranslateUi(QDialog *WBWalkDialog)
    {
        WBWalkDialog->setWindowTitle(QApplication::translate("WBWalkDialog", "Dialog", 0));
        BTN_DATA_SAVE->setText(QApplication::translate("WBWalkDialog", "Data Save", 0));
        BTN_SAVE_START->setText(QApplication::translate("WBWalkDialog", "Save Start", 0));
        BTN_Walk_Ready->setText(QApplication::translate("WBWalkDialog", "Walk Ready", 0));
        BTN_FORWARD->setText(QApplication::translate("WBWalkDialog", "Forward", 0));
        BTN_CW->setText(QApplication::translate("WBWalkDialog", "--CW Right", 0));
        BTN_LEFT->setText(QApplication::translate("WBWalkDialog", "LEFT", 0));
        BTN_RIGHT->setText(QApplication::translate("WBWalkDialog", "RIGHT", 0));
        BTN_CCW->setText(QApplication::translate("WBWalkDialog", "++CCW LEFT", 0));
        label->setText(QApplication::translate("WBWalkDialog", "Step_Num :", 0));
        LE_STEP_NUM->setText(QApplication::translate("WBWalkDialog", "10", 0));
        label_2->setText(QApplication::translate("WBWalkDialog", "Step_Len :", 0));
        LE_STEP_LENGTH->setText(QApplication::translate("WBWalkDialog", "0.001", 0));
        label_4->setText(QApplication::translate("WBWalkDialog", "Step_Offset :", 0));
        LE_STEP_OFFSET->setText(QApplication::translate("WBWalkDialog", "0.25", 0));
        label_3->setText(QApplication::translate("WBWalkDialog", "Step_Ang :", 0));
        LE_STEP_ANGLE->setText(QApplication::translate("WBWalkDialog", "15", 0));
        PB_APPROACH_H->setText(QApplication::translate("WBWalkDialog", "Approach Handle", 0));
        PB_GRASP_H->setText(QApplication::translate("WBWalkDialog", "Grasp Handle", 0));
        PB_QUIT_H->setText(QApplication::translate("WBWalkDialog", "Quit Handle", 0));
    } // retranslateUi

};

namespace Ui {
    class WBWalkDialog: public Ui_WBWalkDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_WBWALKDIALOG_H
