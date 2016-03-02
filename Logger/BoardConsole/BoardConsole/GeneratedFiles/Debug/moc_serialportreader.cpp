/****************************************************************************
** Meta object code from reading C++ file 'serialportreader.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.5.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../serialportreader.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'serialportreader.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.5.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_SerialPortReader_t {
    QByteArrayData data[10];
    char stringdata0[121];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_SerialPortReader_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_SerialPortReader_t qt_meta_stringdata_SerialPortReader = {
    {
QT_MOC_LITERAL(0, 0, 16), // "SerialPortReader"
QT_MOC_LITERAL(1, 17, 9), // "freshLine"
QT_MOC_LITERAL(2, 27, 0), // ""
QT_MOC_LITERAL(3, 28, 8), // "QString&"
QT_MOC_LITERAL(4, 37, 12), // "freshMessage"
QT_MOC_LITERAL(5, 50, 7), // "Message"
QT_MOC_LITERAL(6, 58, 15), // "handleReadyRead"
QT_MOC_LITERAL(7, 74, 11), // "handleError"
QT_MOC_LITERAL(8, 86, 28), // "QSerialPort::SerialPortError"
QT_MOC_LITERAL(9, 115, 5) // "error"

    },
    "SerialPortReader\0freshLine\0\0QString&\0"
    "freshMessage\0Message\0handleReadyRead\0"
    "handleError\0QSerialPort::SerialPortError\0"
    "error"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_SerialPortReader[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    1,   34,    2, 0x06 /* Public */,
       4,    1,   37,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       6,    0,   40,    2, 0x08 /* Private */,
       7,    1,   41,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void, 0x80000000 | 3,    2,
    QMetaType::Void, 0x80000000 | 5,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 8,    9,

       0        // eod
};

void SerialPortReader::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        SerialPortReader *_t = static_cast<SerialPortReader *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->freshLine((*reinterpret_cast< QString(*)>(_a[1]))); break;
        case 1: _t->freshMessage((*reinterpret_cast< Message(*)>(_a[1]))); break;
        case 2: _t->handleReadyRead(); break;
        case 3: _t->handleError((*reinterpret_cast< QSerialPort::SerialPortError(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (SerialPortReader::*_t)(QString & );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&SerialPortReader::freshLine)) {
                *result = 0;
            }
        }
        {
            typedef void (SerialPortReader::*_t)(Message );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&SerialPortReader::freshMessage)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject SerialPortReader::staticMetaObject = {
    { &QObject::staticMetaObject, qt_meta_stringdata_SerialPortReader.data,
      qt_meta_data_SerialPortReader,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *SerialPortReader::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *SerialPortReader::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_SerialPortReader.stringdata0))
        return static_cast<void*>(const_cast< SerialPortReader*>(this));
    return QObject::qt_metacast(_clname);
}

int SerialPortReader::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QObject::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 4)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 4;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 4)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 4;
    }
    return _id;
}

// SIGNAL 0
void SerialPortReader::freshLine(QString & _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 0, _a);
}

// SIGNAL 1
void SerialPortReader::freshMessage(Message _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
