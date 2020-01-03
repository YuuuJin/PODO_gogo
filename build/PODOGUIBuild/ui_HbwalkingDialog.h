/********************************************************************************
** Form generated from reading UI file 'HbwalkingDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_HBWALKINGDIALOG_H
#define UI_HBWALKINGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>

QT_BEGIN_NAMESPACE

class Ui_HBWalkingDialog
{
public:
    QPushButton *BT_ONE_LEG_READY;
    QLabel *label_3;
    QLineEdit *LE_STEP_TIME;
    QPushButton *BT_WALK_TEST;
    QPushButton *BT_TEST_STOP;
    QLineEdit *LE_STEP_STRIDE;
    QPushButton *BT_ONE_LEG_READY_2;
    QPushButton *BT_TEST;
    QPushButton *BT_HOME_POS;
    QLabel *label_4;
    QPushButton *BT_WALK_READY;
    QLabel *label_2;
    QLineEdit *LE_NO_OF_STEP;
    QPushButton *BT_DATA_SAVE;
    QPushButton *BT_READY_TO_WALK;

    void setupUi(QDialog *HBWalkingDialog)
    {
        if (HBWalkingDialog->objectName().isEmpty())
            HBWalkingDialog->setObjectName(QStringLiteral("HBWalkingDialog"));
        HBWalkingDialog->resize(864, 632);
        BT_ONE_LEG_READY = new QPushButton(HBWalkingDialog);
        BT_ONE_LEG_READY->setObjectName(QStringLiteral("BT_ONE_LEG_READY"));
        BT_ONE_LEG_READY->setGeometry(QRect(320, 20, 151, 31));
        label_3 = new QLabel(HBWalkingDialog);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(270, 100, 91, 17));
        LE_STEP_TIME = new QLineEdit(HBWalkingDialog);
        LE_STEP_TIME->setObjectName(QStringLiteral("LE_STEP_TIME"));
        LE_STEP_TIME->setGeometry(QRect(270, 120, 91, 27));
        BT_WALK_TEST = new QPushButton(HBWalkingDialog);
        BT_WALK_TEST->setObjectName(QStringLiteral("BT_WALK_TEST"));
        BT_WALK_TEST->setGeometry(QRect(150, 160, 171, 51));
        BT_TEST_STOP = new QPushButton(HBWalkingDialog);
        BT_TEST_STOP->setObjectName(QStringLiteral("BT_TEST_STOP"));
        BT_TEST_STOP->setGeometry(QRect(30, 170, 99, 27));
        LE_STEP_STRIDE = new QLineEdit(HBWalkingDialog);
        LE_STEP_STRIDE->setObjectName(QStringLiteral("LE_STEP_STRIDE"));
        LE_STEP_STRIDE->setGeometry(QRect(370, 120, 91, 27));
        BT_ONE_LEG_READY_2 = new QPushButton(HBWalkingDialog);
        BT_ONE_LEG_READY_2->setObjectName(QStringLiteral("BT_ONE_LEG_READY_2"));
        BT_ONE_LEG_READY_2->setGeometry(QRect(320, 60, 151, 31));
        BT_TEST = new QPushButton(HBWalkingDialog);
        BT_TEST->setObjectName(QStringLiteral("BT_TEST"));
        BT_TEST->setGeometry(QRect(330, 160, 141, 41));
        BT_HOME_POS = new QPushButton(HBWalkingDialog);
        BT_HOME_POS->setObjectName(QStringLiteral("BT_HOME_POS"));
        BT_HOME_POS->setGeometry(QRect(20, 50, 141, 41));
        label_4 = new QLabel(HBWalkingDialog);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(370, 100, 101, 17));
        BT_WALK_READY = new QPushButton(HBWalkingDialog);
        BT_WALK_READY->setObjectName(QStringLiteral("BT_WALK_READY"));
        BT_WALK_READY->setGeometry(QRect(170, 50, 141, 41));
        label_2 = new QLabel(HBWalkingDialog);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(170, 100, 81, 17));
        LE_NO_OF_STEP = new QLineEdit(HBWalkingDialog);
        LE_NO_OF_STEP->setObjectName(QStringLiteral("LE_NO_OF_STEP"));
        LE_NO_OF_STEP->setGeometry(QRect(170, 120, 91, 27));
        BT_DATA_SAVE = new QPushButton(HBWalkingDialog);
        BT_DATA_SAVE->setObjectName(QStringLiteral("BT_DATA_SAVE"));
        BT_DATA_SAVE->setGeometry(QRect(30, 130, 99, 27));
        BT_READY_TO_WALK = new QPushButton(HBWalkingDialog);
        BT_READY_TO_WALK->setObjectName(QStringLiteral("BT_READY_TO_WALK"));
        BT_READY_TO_WALK->setGeometry(QRect(150, 220, 171, 51));

        retranslateUi(HBWalkingDialog);

        QMetaObject::connectSlotsByName(HBWalkingDialog);
    } // setupUi

    void retranslateUi(QDialog *HBWalkingDialog)
    {
        HBWalkingDialog->setWindowTitle(QApplication::translate("HBWalkingDialog", "Dialog", 0));
        BT_ONE_LEG_READY->setText(QApplication::translate("HBWalkingDialog", "Right Stance", 0));
        label_3->setText(QApplication::translate("HBWalkingDialog", "Step Time(s)", 0));
        LE_STEP_TIME->setText(QApplication::translate("HBWalkingDialog", "0.8", 0));
        BT_WALK_TEST->setText(QApplication::translate("HBWalkingDialog", "Position Walking", 0));
        BT_TEST_STOP->setText(QApplication::translate("HBWalkingDialog", "Stop", 0));
        LE_STEP_STRIDE->setText(QApplication::translate("HBWalkingDialog", "0", 0));
        BT_ONE_LEG_READY_2->setText(QApplication::translate("HBWalkingDialog", "Left Stance", 0));
        BT_TEST->setText(QApplication::translate("HBWalkingDialog", "Test", 0));
        BT_HOME_POS->setText(QApplication::translate("HBWalkingDialog", "Home Pos", 0));
        label_4->setText(QApplication::translate("HBWalkingDialog", "Step Stride(m)", 0));
        BT_WALK_READY->setText(QApplication::translate("HBWalkingDialog", "Walk Ready", 0));
        label_2->setText(QApplication::translate("HBWalkingDialog", "No Of Step", 0));
        LE_NO_OF_STEP->setText(QApplication::translate("HBWalkingDialog", "5", 0));
        BT_DATA_SAVE->setText(QApplication::translate("HBWalkingDialog", "data save", 0));
        BT_READY_TO_WALK->setText(QApplication::translate("HBWalkingDialog", "Ready to Walk", 0));
    } // retranslateUi

};

namespace Ui {
    class HBWalkingDialog: public Ui_HBWalkingDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_HBWALKINGDIALOG_H
