/****************************************************************************
** Meta object code from reading C++ file 'GLGeometryViewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../GLGeometryViewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'GLGeometryViewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_GLGeometryViewer_t {
    QByteArrayData data[11];
    char stringdata[181];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_GLGeometryViewer_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_GLGeometryViewer_t qt_meta_stringdata_GLGeometryViewer = {
    {
QT_MOC_LITERAL(0, 0, 16), // "GLGeometryViewer"
QT_MOC_LITERAL(1, 17, 20), // "viewerContentChanged"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 13), // "pointSelected"
QT_MOC_LITERAL(4, 53, 20), // "on_btnCamera_pressed"
QT_MOC_LITERAL(5, 74, 18), // "on_btnPick_pressed"
QT_MOC_LITERAL(6, 93, 19), // "on_btnClear_pressed"
QT_MOC_LITERAL(7, 113, 20), // "on_btnCreate_pressed"
QT_MOC_LITERAL(8, 134, 23), // "sltWidgetContentChanged"
QT_MOC_LITERAL(9, 158, 16), // "sltPointSelected"
QT_MOC_LITERAL(10, 175, 5) // "index"

    },
    "GLGeometryViewer\0viewerContentChanged\0"
    "\0pointSelected\0on_btnCamera_pressed\0"
    "on_btnPick_pressed\0on_btnClear_pressed\0"
    "on_btnCreate_pressed\0sltWidgetContentChanged\0"
    "sltPointSelected\0index"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_GLGeometryViewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       8,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       2,       // signalCount

 // signals: name, argc, parameters, tag, flags
       1,    0,   54,    2, 0x06 /* Public */,
       3,    1,   55,    2, 0x06 /* Public */,

 // slots: name, argc, parameters, tag, flags
       4,    0,   58,    2, 0x08 /* Private */,
       5,    0,   59,    2, 0x08 /* Private */,
       6,    0,   60,    2, 0x08 /* Private */,
       7,    0,   61,    2, 0x08 /* Private */,
       8,    0,   62,    2, 0x08 /* Private */,
       9,    1,   63,    2, 0x08 /* Private */,

 // signals: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,    2,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Int,   10,

       0        // eod
};

void GLGeometryViewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        GLGeometryViewer *_t = static_cast<GLGeometryViewer *>(_o);
        switch (_id) {
        case 0: _t->viewerContentChanged(); break;
        case 1: _t->pointSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        case 2: _t->on_btnCamera_pressed(); break;
        case 3: _t->on_btnPick_pressed(); break;
        case 4: _t->on_btnClear_pressed(); break;
        case 5: _t->on_btnCreate_pressed(); break;
        case 6: _t->sltWidgetContentChanged(); break;
        case 7: _t->sltPointSelected((*reinterpret_cast< int(*)>(_a[1]))); break;
        default: ;
        }
    } else if (_c == QMetaObject::IndexOfMethod) {
        int *result = reinterpret_cast<int *>(_a[0]);
        void **func = reinterpret_cast<void **>(_a[1]);
        {
            typedef void (GLGeometryViewer::*_t)();
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLGeometryViewer::viewerContentChanged)) {
                *result = 0;
            }
        }
        {
            typedef void (GLGeometryViewer::*_t)(int );
            if (*reinterpret_cast<_t *>(func) == static_cast<_t>(&GLGeometryViewer::pointSelected)) {
                *result = 1;
            }
        }
    }
}

const QMetaObject GLGeometryViewer::staticMetaObject = {
    { &QWidget::staticMetaObject, qt_meta_stringdata_GLGeometryViewer.data,
      qt_meta_data_GLGeometryViewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *GLGeometryViewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *GLGeometryViewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_GLGeometryViewer.stringdata))
        return static_cast<void*>(const_cast< GLGeometryViewer*>(this));
    return QWidget::qt_metacast(_clname);
}

int GLGeometryViewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
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

// SIGNAL 0
void GLGeometryViewer::viewerContentChanged()
{
    QMetaObject::activate(this, &staticMetaObject, 0, Q_NULLPTR);
}

// SIGNAL 1
void GLGeometryViewer::pointSelected(int _t1)
{
    void *_a[] = { Q_NULLPTR, const_cast<void*>(reinterpret_cast<const void*>(&_t1)) };
    QMetaObject::activate(this, &staticMetaObject, 1, _a);
}
QT_END_MOC_NAMESPACE
