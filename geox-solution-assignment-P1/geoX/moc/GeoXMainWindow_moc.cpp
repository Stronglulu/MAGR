/****************************************************************************
** Meta object code from reading C++ file 'GeoXMainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../GeoXMainWindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GeoXMainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_GeoXMainWindow_t {
    QByteArrayData data[9];
    char stringdata[164];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GeoXMainWindow_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GeoXMainWindow_t qt_meta_stringdata_GeoXMainWindow = {
    {
QT_MOC_LITERAL(0, 0, 14), // "GeoXMainWindow"
QT_MOC_LITERAL(1, 15, 23), // "on_actionOpen_triggered"
QT_MOC_LITERAL(2, 39, 0), // ""
QT_MOC_LITERAL(3, 40, 25), // "on_actionSaveAs_triggered"
QT_MOC_LITERAL(4, 66, 23), // "on_actionExit_triggered"
QT_MOC_LITERAL(5, 90, 10), // "initWindow"
QT_MOC_LITERAL(6, 101, 20), // "sltExperimentChanged"
QT_MOC_LITERAL(7, 122, 17), // "newExperimentName"
QT_MOC_LITERAL(8, 140, 23) // "sltViewerContentChanged"

    },
    "GeoXMainWindow\0on_actionOpen_triggered\0"
    "\0on_actionSaveAs_triggered\0"
    "on_actionExit_triggered\0initWindow\0"
    "sltExperimentChanged\0newExperimentName\0"
    "sltViewerContentChanged"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GeoXMainWindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       6,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   44,    2, 0x08 /* Private */,
       3,    0,   45,    2, 0x08 /* Private */,
       4,    0,   46,    2, 0x08 /* Private */,
       5,    0,   47,    2, 0x08 /* Private */,
       6,    1,   48,    2, 0x08 /* Private */,
       8,    0,   51,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QString,    7,
    QMetaType::Void,

       0        // eod
};

void GeoXMainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GeoXMainWindow *_t = static_cast<GeoXMainWindow *>(_o);
        switch (_id) {
        case 0: _t->on_actionOpen_triggered(); break;
        case 1: _t->on_actionSaveAs_triggered(); break;
        case 2: _t->on_actionExit_triggered(); break;
        case 3: _t->initWindow(); break;
        case 4: _t->sltExperimentChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 5: _t->sltViewerContentChanged(); break;
        default: ;
        }
    }
}

const QMetaObject GeoXMainWindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_GeoXMainWindow.data,
      qt_meta_data_GeoXMainWindow,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *GeoXMainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GeoXMainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_GeoXMainWindow.stringdata))
        return static_cast<void*>(const_cast< GeoXMainWindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int GeoXMainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 6)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 6;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 6)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 6;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
