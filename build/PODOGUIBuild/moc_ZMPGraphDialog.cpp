/****************************************************************************
** Meta object code from reading C++ file 'ZMPGraphDialog.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../src/PODOGUI/ZMPGraphDialog.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ZMPGraphDialog.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ZMPGraphDialog_t {
    QByteArrayData data[10];
    char stringdata0[193];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ZMPGraphDialog_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ZMPGraphDialog_t qt_meta_stringdata_ZMPGraphDialog = {
    {
QT_MOC_LITERAL(0, 0, 14), // "ZMPGraphDialog"
QT_MOC_LITERAL(1, 15, 16), // "RealTimeDataSlot"
QT_MOC_LITERAL(2, 32, 0), // ""
QT_MOC_LITERAL(3, 33, 7), // "AddData"
QT_MOC_LITERAL(4, 41, 30), // "on_PB_COMPLIANCE_START_clicked"
QT_MOC_LITERAL(5, 72, 16), // "Plot_GRP_ZMPFOOT"
QT_MOC_LITERAL(6, 89, 27), // "on_PB_CLEAR_ZMPFOOT_clicked"
QT_MOC_LITERAL(7, 117, 29), // "on_PB_COMPLIANCE_STOP_clicked"
QT_MOC_LITERAL(8, 147, 21), // "on_pushButton_clicked"
QT_MOC_LITERAL(9, 169, 23) // "on_pushButton_2_clicked"

    },
    "ZMPGraphDialog\0RealTimeDataSlot\0\0"
    "AddData\0on_PB_COMPLIANCE_START_clicked\0"
    "Plot_GRP_ZMPFOOT\0on_PB_CLEAR_ZMPFOOT_clicked\0"
    "on_PB_COMPLIANCE_STOP_clicked\0"
    "on_pushButton_clicked\0on_pushButton_2_clicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ZMPGraphDialog[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x08 /* Private */,
       3,    0,   55,    2, 0x08 /* Private */,
       4,    0,   56,    2, 0x08 /* Private */,
       5,    0,   57,    2, 0x08 /* Private */,
       6,    0,   58,    2, 0x08 /* Private */,
       7,    0,   59,    2, 0x08 /* Private */,
       8,    0,   60,    2, 0x08 /* Private */,
       9,    0,   61,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void ZMPGraphDialog::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ZMPGraphDialog *_t = static_cast<ZMPGraphDialog *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->RealTimeDataSlot(); break;
        case 1: _t->AddData(); break;
        case 2: _t->on_PB_COMPLIANCE_START_clicked(); break;
        case 3: _t->Plot_GRP_ZMPFOOT(); break;
        case 4: _t->on_PB_CLEAR_ZMPFOOT_clicked(); break;
        case 5: _t->on_PB_COMPLIANCE_STOP_clicked(); break;
        case 6: _t->on_pushButton_clicked(); break;
        case 7: _t->on_pushButton_2_clicked(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject ZMPGraphDialog::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ZMPGraphDialog.data,
      qt_meta_data_ZMPGraphDialog,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ZMPGraphDialog::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ZMPGraphDialog::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ZMPGraphDialog.stringdata0))
        return static_cast<void*>(const_cast< ZMPGraphDialog*>(this));
    return QDialog::qt_metacast(_clname);
}

int ZMPGraphDialog::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 8)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 8;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 8)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 8;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
