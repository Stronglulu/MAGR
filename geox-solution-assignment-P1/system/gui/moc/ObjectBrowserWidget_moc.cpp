/****************************************************************************
** Meta object code from reading C++ file 'ObjectBrowserWidget.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ObjectBrowserWidget.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ObjectBrowserWidget.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ObjectBrowserWidget_t {
    QByteArrayData data[8];
    char stringdata[85];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ObjectBrowserWidget_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ObjectBrowserWidget_t qt_meta_stringdata_ObjectBrowserWidget = {
    {
QT_MOC_LITERAL(0, 0, 19), // "ObjectBrowserWidget"
QT_MOC_LITERAL(1, 20, 13), // "sltHandleMenu"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 8), // "QAction*"
QT_MOC_LITERAL(4, 44, 6), // "action"
QT_MOC_LITERAL(5, 51, 21), // "on_editButton_clicked"
QT_MOC_LITERAL(6, 73, 9), // "sltUpdate"
QT_MOC_LITERAL(7, 83, 1) // "x"

    },
    "ObjectBrowserWidget\0sltHandleMenu\0\0"
    "QAction*\0action\0on_editButton_clicked\0"
    "sltUpdate\0x"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ObjectBrowserWidget[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       3,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   29,    2, 0x0a /* Public */,
       5,    0,   32,    2, 0x0a /* Public */,
       6,    1,   33,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, 0x80000000 | 3,    4,
    QMetaType::Void,
    QMetaType::Void, QMetaType::Bool,    7,

       0        // eod
};

void ObjectBrowserWidget::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ObjectBrowserWidget *_t = static_cast<ObjectBrowserWidget *>(_o);
        switch (_id) {
        case 0: _t->sltHandleMenu((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 1: _t->on_editButton_clicked(); break;
        case 2: _t->sltUpdate((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject ObjectBrowserWidget::staticMetaObject = {
    { &QDialog::staticMetaObject, qt_meta_stringdata_ObjectBrowserWidget.data,
      qt_meta_data_ObjectBrowserWidget,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ObjectBrowserWidget::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ObjectBrowserWidget::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ObjectBrowserWidget.stringdata))
        return static_cast<void*>(const_cast< ObjectBrowserWidget*>(this));
    return QDialog::qt_metacast(_clname);
}

int ObjectBrowserWidget::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QDialog::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 3)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 3;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 3)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 3;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
