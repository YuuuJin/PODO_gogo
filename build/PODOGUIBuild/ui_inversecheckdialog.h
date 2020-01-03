/********************************************************************************
** Form generated from reading UI file 'inversecheckdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_INVERSECHECKDIALOG_H
#define UI_INVERSECHECKDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_InverseCheckDialog
{
public:
    QLabel *label_20;
    QLabel *label_21;
    QWidget *layoutWidget;
    QHBoxLayout *horizontalLayout_6;
    QVBoxLayout *verticalLayout_11;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_6;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QVBoxLayout *verticalLayout;
    QLineEdit *LE_InputX;
    QLineEdit *LE_InputY;
    QLineEdit *LE_InputZ;
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout_7;
    QLabel *label_4;
    QLabel *label_5;
    QLabel *label_6;
    QVBoxLayout *verticalLayout_2;
    QLineEdit *LE_InputRoll;
    QLineEdit *LE_InputPitch;
    QLineEdit *LE_InputYaw;
    QPushButton *PB_IKandGO;
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout_10;
    QLabel *label_14;
    QLabel *label_15;
    QLabel *label_13;
    QLabel *label_16;
    QLabel *label_17;
    QLabel *label_18;
    QLabel *label_19;
    QVBoxLayout *verticalLayout_3;
    QLineEdit *LE_SP;
    QLineEdit *LE_SR;
    QLineEdit *LE_SY;
    QLineEdit *LE_EB;
    QLineEdit *LE_WY;
    QLineEdit *LE_WY2;
    QLineEdit *LE_WP;
    QPushButton *PB_FK;
    QVBoxLayout *verticalLayout_12;
    QHBoxLayout *horizontalLayout_3;
    QVBoxLayout *verticalLayout_8;
    QLabel *label_7;
    QLabel *label_8;
    QLabel *label_9;
    QVBoxLayout *verticalLayout_5;
    QLineEdit *LE_OutputX;
    QLineEdit *LE_OutputY;
    QLineEdit *LE_OutputZ;
    QHBoxLayout *horizontalLayout_4;
    QVBoxLayout *verticalLayout_9;
    QLabel *label_10;
    QLabel *label_11;
    QLabel *label_12;
    QVBoxLayout *verticalLayout_4;
    QLineEdit *LE_OutputRoll;
    QLineEdit *LE_OutputPitch;
    QLineEdit *LE_OutputYaw;
    QPushButton *PB_WALKREADY;
    QWidget *widget;
    QVBoxLayout *verticalLayout_13;
    QCheckBox *CB_X;
    QCheckBox *CB_Y;
    QCheckBox *CB_Z;
    QCheckBox *CB_R;
    QCheckBox *CB_P;
    QCheckBox *CB_Yaw;

    void setupUi(QDialog *InverseCheckDialog)
    {
        if (InverseCheckDialog->objectName().isEmpty())
            InverseCheckDialog->setObjectName(QStringLiteral("InverseCheckDialog"));
        InverseCheckDialog->resize(773, 520);
        label_20 = new QLabel(InverseCheckDialog);
        label_20->setObjectName(QStringLiteral("label_20"));
        label_20->setGeometry(QRect(80, 20, 59, 14));
        label_21 = new QLabel(InverseCheckDialog);
        label_21->setObjectName(QStringLiteral("label_21"));
        label_21->setGeometry(QRect(630, 20, 59, 14));
        layoutWidget = new QWidget(InverseCheckDialog);
        layoutWidget->setObjectName(QStringLiteral("layoutWidget"));
        layoutWidget->setGeometry(QRect(10, 40, 741, 196));
        horizontalLayout_6 = new QHBoxLayout(layoutWidget);
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        horizontalLayout_6->setContentsMargins(0, 0, 0, 0);
        verticalLayout_11 = new QVBoxLayout();
        verticalLayout_11->setObjectName(QStringLiteral("verticalLayout_11"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout_6 = new QVBoxLayout();
        verticalLayout_6->setObjectName(QStringLiteral("verticalLayout_6"));
        label = new QLabel(layoutWidget);
        label->setObjectName(QStringLiteral("label"));

        verticalLayout_6->addWidget(label);

        label_2 = new QLabel(layoutWidget);
        label_2->setObjectName(QStringLiteral("label_2"));

        verticalLayout_6->addWidget(label_2);

        label_3 = new QLabel(layoutWidget);
        label_3->setObjectName(QStringLiteral("label_3"));

        verticalLayout_6->addWidget(label_3);


        horizontalLayout->addLayout(verticalLayout_6);

        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        LE_InputX = new QLineEdit(layoutWidget);
        LE_InputX->setObjectName(QStringLiteral("LE_InputX"));

        verticalLayout->addWidget(LE_InputX);

        LE_InputY = new QLineEdit(layoutWidget);
        LE_InputY->setObjectName(QStringLiteral("LE_InputY"));

        verticalLayout->addWidget(LE_InputY);

        LE_InputZ = new QLineEdit(layoutWidget);
        LE_InputZ->setObjectName(QStringLiteral("LE_InputZ"));

        verticalLayout->addWidget(LE_InputZ);


        horizontalLayout->addLayout(verticalLayout);


        verticalLayout_11->addLayout(horizontalLayout);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        verticalLayout_7 = new QVBoxLayout();
        verticalLayout_7->setObjectName(QStringLiteral("verticalLayout_7"));
        label_4 = new QLabel(layoutWidget);
        label_4->setObjectName(QStringLiteral("label_4"));

        verticalLayout_7->addWidget(label_4);

        label_5 = new QLabel(layoutWidget);
        label_5->setObjectName(QStringLiteral("label_5"));

        verticalLayout_7->addWidget(label_5);

        label_6 = new QLabel(layoutWidget);
        label_6->setObjectName(QStringLiteral("label_6"));

        verticalLayout_7->addWidget(label_6);


        horizontalLayout_2->addLayout(verticalLayout_7);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        LE_InputRoll = new QLineEdit(layoutWidget);
        LE_InputRoll->setObjectName(QStringLiteral("LE_InputRoll"));

        verticalLayout_2->addWidget(LE_InputRoll);

        LE_InputPitch = new QLineEdit(layoutWidget);
        LE_InputPitch->setObjectName(QStringLiteral("LE_InputPitch"));

        verticalLayout_2->addWidget(LE_InputPitch);

        LE_InputYaw = new QLineEdit(layoutWidget);
        LE_InputYaw->setObjectName(QStringLiteral("LE_InputYaw"));

        verticalLayout_2->addWidget(LE_InputYaw);


        horizontalLayout_2->addLayout(verticalLayout_2);


        verticalLayout_11->addLayout(horizontalLayout_2);


        horizontalLayout_6->addLayout(verticalLayout_11);

        PB_IKandGO = new QPushButton(layoutWidget);
        PB_IKandGO->setObjectName(QStringLiteral("PB_IKandGO"));

        horizontalLayout_6->addWidget(PB_IKandGO);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        verticalLayout_10 = new QVBoxLayout();
        verticalLayout_10->setObjectName(QStringLiteral("verticalLayout_10"));
        label_14 = new QLabel(layoutWidget);
        label_14->setObjectName(QStringLiteral("label_14"));

        verticalLayout_10->addWidget(label_14);

        label_15 = new QLabel(layoutWidget);
        label_15->setObjectName(QStringLiteral("label_15"));

        verticalLayout_10->addWidget(label_15);

        label_13 = new QLabel(layoutWidget);
        label_13->setObjectName(QStringLiteral("label_13"));

        verticalLayout_10->addWidget(label_13);

        label_16 = new QLabel(layoutWidget);
        label_16->setObjectName(QStringLiteral("label_16"));

        verticalLayout_10->addWidget(label_16);

        label_17 = new QLabel(layoutWidget);
        label_17->setObjectName(QStringLiteral("label_17"));

        verticalLayout_10->addWidget(label_17);

        label_18 = new QLabel(layoutWidget);
        label_18->setObjectName(QStringLiteral("label_18"));

        verticalLayout_10->addWidget(label_18);

        label_19 = new QLabel(layoutWidget);
        label_19->setObjectName(QStringLiteral("label_19"));

        verticalLayout_10->addWidget(label_19);


        horizontalLayout_5->addLayout(verticalLayout_10);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        LE_SP = new QLineEdit(layoutWidget);
        LE_SP->setObjectName(QStringLiteral("LE_SP"));

        verticalLayout_3->addWidget(LE_SP);

        LE_SR = new QLineEdit(layoutWidget);
        LE_SR->setObjectName(QStringLiteral("LE_SR"));

        verticalLayout_3->addWidget(LE_SR);

        LE_SY = new QLineEdit(layoutWidget);
        LE_SY->setObjectName(QStringLiteral("LE_SY"));

        verticalLayout_3->addWidget(LE_SY);

        LE_EB = new QLineEdit(layoutWidget);
        LE_EB->setObjectName(QStringLiteral("LE_EB"));

        verticalLayout_3->addWidget(LE_EB);

        LE_WY = new QLineEdit(layoutWidget);
        LE_WY->setObjectName(QStringLiteral("LE_WY"));

        verticalLayout_3->addWidget(LE_WY);

        LE_WY2 = new QLineEdit(layoutWidget);
        LE_WY2->setObjectName(QStringLiteral("LE_WY2"));

        verticalLayout_3->addWidget(LE_WY2);

        LE_WP = new QLineEdit(layoutWidget);
        LE_WP->setObjectName(QStringLiteral("LE_WP"));

        verticalLayout_3->addWidget(LE_WP);


        horizontalLayout_5->addLayout(verticalLayout_3);


        horizontalLayout_6->addLayout(horizontalLayout_5);

        PB_FK = new QPushButton(layoutWidget);
        PB_FK->setObjectName(QStringLiteral("PB_FK"));

        horizontalLayout_6->addWidget(PB_FK);

        verticalLayout_12 = new QVBoxLayout();
        verticalLayout_12->setObjectName(QStringLiteral("verticalLayout_12"));
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        verticalLayout_8 = new QVBoxLayout();
        verticalLayout_8->setObjectName(QStringLiteral("verticalLayout_8"));
        label_7 = new QLabel(layoutWidget);
        label_7->setObjectName(QStringLiteral("label_7"));

        verticalLayout_8->addWidget(label_7);

        label_8 = new QLabel(layoutWidget);
        label_8->setObjectName(QStringLiteral("label_8"));

        verticalLayout_8->addWidget(label_8);

        label_9 = new QLabel(layoutWidget);
        label_9->setObjectName(QStringLiteral("label_9"));

        verticalLayout_8->addWidget(label_9);


        horizontalLayout_3->addLayout(verticalLayout_8);

        verticalLayout_5 = new QVBoxLayout();
        verticalLayout_5->setObjectName(QStringLiteral("verticalLayout_5"));
        LE_OutputX = new QLineEdit(layoutWidget);
        LE_OutputX->setObjectName(QStringLiteral("LE_OutputX"));

        verticalLayout_5->addWidget(LE_OutputX);

        LE_OutputY = new QLineEdit(layoutWidget);
        LE_OutputY->setObjectName(QStringLiteral("LE_OutputY"));

        verticalLayout_5->addWidget(LE_OutputY);

        LE_OutputZ = new QLineEdit(layoutWidget);
        LE_OutputZ->setObjectName(QStringLiteral("LE_OutputZ"));

        verticalLayout_5->addWidget(LE_OutputZ);


        horizontalLayout_3->addLayout(verticalLayout_5);


        verticalLayout_12->addLayout(horizontalLayout_3);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        verticalLayout_9 = new QVBoxLayout();
        verticalLayout_9->setObjectName(QStringLiteral("verticalLayout_9"));
        label_10 = new QLabel(layoutWidget);
        label_10->setObjectName(QStringLiteral("label_10"));

        verticalLayout_9->addWidget(label_10);

        label_11 = new QLabel(layoutWidget);
        label_11->setObjectName(QStringLiteral("label_11"));

        verticalLayout_9->addWidget(label_11);

        label_12 = new QLabel(layoutWidget);
        label_12->setObjectName(QStringLiteral("label_12"));

        verticalLayout_9->addWidget(label_12);


        horizontalLayout_4->addLayout(verticalLayout_9);

        verticalLayout_4 = new QVBoxLayout();
        verticalLayout_4->setObjectName(QStringLiteral("verticalLayout_4"));
        LE_OutputRoll = new QLineEdit(layoutWidget);
        LE_OutputRoll->setObjectName(QStringLiteral("LE_OutputRoll"));

        verticalLayout_4->addWidget(LE_OutputRoll);

        LE_OutputPitch = new QLineEdit(layoutWidget);
        LE_OutputPitch->setObjectName(QStringLiteral("LE_OutputPitch"));

        verticalLayout_4->addWidget(LE_OutputPitch);

        LE_OutputYaw = new QLineEdit(layoutWidget);
        LE_OutputYaw->setObjectName(QStringLiteral("LE_OutputYaw"));

        verticalLayout_4->addWidget(LE_OutputYaw);


        horizontalLayout_4->addLayout(verticalLayout_4);


        verticalLayout_12->addLayout(horizontalLayout_4);


        horizontalLayout_6->addLayout(verticalLayout_12);

        PB_WALKREADY = new QPushButton(InverseCheckDialog);
        PB_WALKREADY->setObjectName(QStringLiteral("PB_WALKREADY"));
        PB_WALKREADY->setGeometry(QRect(600, 260, 141, 51));
        widget = new QWidget(InverseCheckDialog);
        widget->setObjectName(QStringLiteral("widget"));
        widget->setGeometry(QRect(140, 270, 87, 152));
        verticalLayout_13 = new QVBoxLayout(widget);
        verticalLayout_13->setObjectName(QStringLiteral("verticalLayout_13"));
        verticalLayout_13->setContentsMargins(0, 0, 0, 0);
        CB_X = new QCheckBox(widget);
        CB_X->setObjectName(QStringLiteral("CB_X"));

        verticalLayout_13->addWidget(CB_X);

        CB_Y = new QCheckBox(widget);
        CB_Y->setObjectName(QStringLiteral("CB_Y"));

        verticalLayout_13->addWidget(CB_Y);

        CB_Z = new QCheckBox(widget);
        CB_Z->setObjectName(QStringLiteral("CB_Z"));

        verticalLayout_13->addWidget(CB_Z);

        CB_R = new QCheckBox(widget);
        CB_R->setObjectName(QStringLiteral("CB_R"));

        verticalLayout_13->addWidget(CB_R);

        CB_P = new QCheckBox(widget);
        CB_P->setObjectName(QStringLiteral("CB_P"));

        verticalLayout_13->addWidget(CB_P);

        CB_Yaw = new QCheckBox(widget);
        CB_Yaw->setObjectName(QStringLiteral("CB_Yaw"));

        verticalLayout_13->addWidget(CB_Yaw);


        retranslateUi(InverseCheckDialog);

        QMetaObject::connectSlotsByName(InverseCheckDialog);
    } // setupUi

    void retranslateUi(QDialog *InverseCheckDialog)
    {
        InverseCheckDialog->setWindowTitle(QApplication::translate("InverseCheckDialog", "Dialog", 0));
        label_20->setText(QApplication::translate("InverseCheckDialog", "INPUT", 0));
        label_21->setText(QApplication::translate("InverseCheckDialog", "OUTPUT", 0));
        label->setText(QApplication::translate("InverseCheckDialog", "X :", 0));
        label_2->setText(QApplication::translate("InverseCheckDialog", "Y :", 0));
        label_3->setText(QApplication::translate("InverseCheckDialog", "Z :", 0));
        label_4->setText(QApplication::translate("InverseCheckDialog", "Roll :", 0));
        label_5->setText(QApplication::translate("InverseCheckDialog", "Pitch :", 0));
        label_6->setText(QApplication::translate("InverseCheckDialog", "Yaw :", 0));
        PB_IKandGO->setText(QApplication::translate("InverseCheckDialog", "IK and Go", 0));
        label_14->setText(QApplication::translate("InverseCheckDialog", "ShoulderPitch :", 0));
        label_15->setText(QApplication::translate("InverseCheckDialog", "ShoulderRoll :", 0));
        label_13->setText(QApplication::translate("InverseCheckDialog", "ShoulderYaw :", 0));
        label_16->setText(QApplication::translate("InverseCheckDialog", "ElbowPitch :", 0));
        label_17->setText(QApplication::translate("InverseCheckDialog", "WristYaw :", 0));
        label_18->setText(QApplication::translate("InverseCheckDialog", "WristYaw2 :", 0));
        label_19->setText(QApplication::translate("InverseCheckDialog", "WristPitch :", 0));
        PB_FK->setText(QApplication::translate("InverseCheckDialog", "FK", 0));
        label_7->setText(QApplication::translate("InverseCheckDialog", "X :", 0));
        label_8->setText(QApplication::translate("InverseCheckDialog", "Y :", 0));
        label_9->setText(QApplication::translate("InverseCheckDialog", "Z :", 0));
        label_10->setText(QApplication::translate("InverseCheckDialog", "Roll :", 0));
        label_11->setText(QApplication::translate("InverseCheckDialog", "Pitch :", 0));
        label_12->setText(QApplication::translate("InverseCheckDialog", "Yaw :", 0));
        PB_WALKREADY->setText(QApplication::translate("InverseCheckDialog", "WalkReady", 0));
        CB_X->setText(QApplication::translate("InverseCheckDialog", "x", 0));
        CB_Y->setText(QApplication::translate("InverseCheckDialog", "y", 0));
        CB_Z->setText(QApplication::translate("InverseCheckDialog", "z", 0));
        CB_R->setText(QApplication::translate("InverseCheckDialog", "roll", 0));
        CB_P->setText(QApplication::translate("InverseCheckDialog", "pitch", 0));
        CB_Yaw->setText(QApplication::translate("InverseCheckDialog", "yaw", 0));
    } // retranslateUi

};

namespace Ui {
    class InverseCheckDialog: public Ui_InverseCheckDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_INVERSECHECKDIALOG_H
