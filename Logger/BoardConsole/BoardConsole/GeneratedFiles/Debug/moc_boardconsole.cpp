/****************************************************************************
** Meta object code from reading C++ file 'boardconsole.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../boardconsole.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'boardconsole.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_BoardConsole_t {
    QByteArrayData data[20];
    char stringdata0[371];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BoardConsole_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BoardConsole_t qt_meta_stringdata_BoardConsole = {
    {
QT_MOC_LITERAL(0, 0, 12), // "BoardConsole"
QT_MOC_LITERAL(1, 13, 19), // "handleConnectButton"
QT_MOC_LITERAL(2, 33, 0), // ""
QT_MOC_LITERAL(3, 34, 22), // "handleStabToggleButton"
QT_MOC_LITERAL(4, 57, 18), // "handleCalibrButton"
QT_MOC_LITERAL(5, 76, 22), // "handleTelemetryButtons"
QT_MOC_LITERAL(6, 99, 27), // "handleTelemetryToggleButton"
QT_MOC_LITERAL(7, 127, 18), // "handleAccelButtons"
QT_MOC_LITERAL(8, 146, 15), // "handleFreshLine"
QT_MOC_LITERAL(9, 162, 8), // "QString&"
QT_MOC_LITERAL(10, 171, 4), // "line"
QT_MOC_LITERAL(11, 176, 22), // "handleNoFilterCheckBox"
QT_MOC_LITERAL(12, 199, 26), // "handleKalmanFilterCheckBox"
QT_MOC_LITERAL(13, 226, 23), // "handleAveragingCheckBox"
QT_MOC_LITERAL(14, 250, 24), // "handleSaveToFileCheckBox"
QT_MOC_LITERAL(15, 275, 14), // "handleK1Button"
QT_MOC_LITERAL(16, 290, 14), // "handleK2Button"
QT_MOC_LITERAL(17, 305, 29), // "handleTelemetryDisplayButtons"
QT_MOC_LITERAL(18, 335, 17), // "handlePwm1SpinBox"
QT_MOC_LITERAL(19, 353, 17) // "handlePwm2SpinBox"

    },
    "BoardConsole\0handleConnectButton\0\0"
    "handleStabToggleButton\0handleCalibrButton\0"
    "handleTelemetryButtons\0"
    "handleTelemetryToggleButton\0"
    "handleAccelButtons\0handleFreshLine\0"
    "QString&\0line\0handleNoFilterCheckBox\0"
    "handleKalmanFilterCheckBox\0"
    "handleAveragingCheckBox\0"
    "handleSaveToFileCheckBox\0handleK1Button\0"
    "handleK2Button\0handleTelemetryDisplayButtons\0"
    "handlePwm1SpinBox\0handlePwm2SpinBox"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BoardConsole[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      16,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   94,    2, 0x08 /* Private */,
       3,    0,   95,    2, 0x08 /* Private */,
       4,    0,   96,    2, 0x08 /* Private */,
       5,    0,   97,    2, 0x08 /* Private */,
       6,    0,   98,    2, 0x08 /* Private */,
       7,    0,   99,    2, 0x08 /* Private */,
       8,    1,  100,    2, 0x08 /* Private */,
      11,    0,  103,    2, 0x08 /* Private */,
      12,    0,  104,    2, 0x08 /* Private */,
      13,    0,  105,    2, 0x08 /* Private */,
      14,    0,  106,    2, 0x08 /* Private */,
      15,    0,  107,    2, 0x08 /* Private */,
      16,    0,  108,    2, 0x08 /* Private */,
      17,    0,  109,    2, 0x08 /* Private */,
      18,    1,  110,    2, 0x08 /* Private */,
      19,    1,  113,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 9,   10,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,
    QMetaType::Void, QMetaType::Int,    2,

       0        // eod
};

void BoardConsole::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BoardConsole *_t = static_cast<BoardConsole *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->handleConnectButton(); break;
        case 1: _t->handleStabToggleButton(); break;
        case 2: _t->handleCalibrButton(); break;
        case 3: _t->handleTelemetryButtons(); break;
        case 4: _t->handleTelemetryToggleButton(); break;
        case 5: _t->handleAccelButtons(); break;
        case 6: _t->handleFreshLine((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 7: _t->handleNoFilterCheckBox(); break;
        case 8: _t->handleKalmanFilterCheckBox(); break;
        case 9: _t->handleAveragingCheckBox(); break;
        case 10: _t->handleSaveToFileCheckBox(); break;
        case 11: _t->handleK1Button(); break;
        case 12: _t->handleK2Button(); break;
        case 13: _t->handleTelemetryDisplayButtons(); break;
        case 14: _t->handlePwm1SpinBox((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 15: _t->handlePwm2SpinBox((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject BoardConsole::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_BoardConsole.data,
      qt_meta_data_BoardConsole,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BoardConsole::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BoardConsole::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BoardConsole.stringdata0))
        return static_cast<void*>(const_cast< BoardConsole*>(this));
    return QWidget::qt_metacast(_clname);
}

int BoardConsole::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 16)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 16;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 16)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 16;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
