/****************************************************************************
** Meta object code from reading C++ file 'BasicGeoXGLWidget3D.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../BasicGeoXGLWidget3D.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'BasicGeoXGLWidget3D.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_BasicGeoXGLWidget3D_t {
    QByteArrayData data[4];
    char stringdata[56];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_BasicGeoXGLWidget3D_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_BasicGeoXGLWidget3D_t qt_meta_stringdata_BasicGeoXGLWidget3D = {
    {
QT_MOC_LITERAL(0, 0, 19), // "BasicGeoXGLWidget3D"
QT_MOC_LITERAL(1, 20, 20), // "widgetContentChanged"
QT_MOC_LITERAL(2, 41, 0), // ""
QT_MOC_LITERAL(3, 42, 13) // "customPaintGL"

    },
    "BasicGeoXGLWidget3D\0widgetContentChanged\0"
    "\0customPaintGL"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_BasicGeoXGLWidget3D[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x06 /* Public */,
       3,    0,   25,    2, 0x06 /* Public */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void BasicGeoXGLWidget3D::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        BasicGeoXGLWidget3D *_t = static_cast<BasicGeoXGLWidget3D *>(_o);
        switch (_id) {
        case 0: _t->widgetContentChanged(); break;
        case 1: _t->customPaintGL(); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (BasicGeoXGLWidget3D::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BasicGeoXGLWidget3D::widgetContentChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (BasicGeoXGLWidget3D::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&BasicGeoXGLWidget3D::customPaintGL)) {
                *result = 1;
            }
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject BasicGeoXGLWidget3D::staticMetaObject = {
    { &QGLWidget::staticMetaObject, qt_meta_stringdata_BasicGeoXGLWidget3D.data,
      qt_meta_data_BasicGeoXGLWidget3D,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *BasicGeoXGLWidget3D::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *BasicGeoXGLWidget3D::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_BasicGeoXGLWidget3D.stringdata))
        return static_cast<void*>(const_cast< BasicGeoXGLWidget3D*>(this));
    return QGLWidget::qt_metacast(_clname);
}

int BasicGeoXGLWidget3D::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLWidget::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 2)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 2;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 2)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 2;
    }
    return _id;
}

// SIGNAL 0
void BasicGeoXGLWidget3D::widgetContentChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void BasicGeoXGLWidget3D::customPaintGL()
{
    QMetaObject::activate(this, &staticMetaObject, 1, Q_NULLPTR);
}
QT_END_MOC_NAMESPACE
