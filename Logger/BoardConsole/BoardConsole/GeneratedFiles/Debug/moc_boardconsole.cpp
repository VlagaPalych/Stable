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
    QByteArrayData data[12];
    char stringdata0[198];
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
QT_MOC_LITERAL(3, 34, 18), // "handleStabOnButton"
QT_MOC_LITERAL(4, 53, 19), // "handleStabOffButton"
QT_MOC_LITERAL(5, 73, 18), // "handleCalibrButton"
QT_MOC_LITERAL(6, 92, 22), // "handleTelemetryButtons"
QT_MOC_LITERAL(7, 115, 26), // "handleTelemetryStartButton"
QT_MOC_LITERAL(8, 142, 25), // "handleTelemetryStopButton"
QT_MOC_LITERAL(9, 168, 15), // "handleFreshLine"
QT_MOC_LITERAL(10, 184, 8), // "QString&"
QT_MOC_LITERAL(11, 193, 4) // "line"

    },
    "BoardConsole\0handleConnectButton\0\0"
    "handleStabOnButton\0handleStabOffButton\0"
    "handleCalibrButton\0handleTelemetryButtons\0"
    "handleTelemetryStartButton\0"
    "handleTelemetryStopButton\0handleFreshLine\0"
    "QString&\0line"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BoardConsole[] = {

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
       9,    1,   61,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 10,   11,

       0        // eod
};

void BoardConsole::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BoardConsole *_t = static_cast<BoardConsole *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->handleConnectButton(); break;
        case 1: _t->handleStabOnButton(); break;
        case 2: _t->handleStabOffButton(); break;
        case 3: _t->handleCalibrButton(); break;
        case 4: _t->handleTelemetryButtons(); break;
        case 5: _t->handleTelemetryStartButton(); break;
        case 6: _t->handleTelemetryStopButton(); break;
        case 7: _t->handleFreshLine((*reinterpret_cast< QString(*)>(_a[1]))); break;
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
