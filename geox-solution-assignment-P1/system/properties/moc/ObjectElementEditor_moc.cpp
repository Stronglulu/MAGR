/****************************************************************************
** Meta object code from reading C++ file 'ObjectElementEditor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ObjectElementEditor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ObjectElementEditor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ObjectElementEditor_t {
    QByteArrayData data[9];
    char stringdata[112];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ObjectElementEditor_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ObjectElementEditor_t qt_meta_stringdata_ObjectElementEditor = {
    {
QT_MOC_LITERAL(0, 0, 19), // "ObjectElementEditor"
QT_MOC_LITERAL(1, 20, 13), // "sltEditButton"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 19), // "sltViewObjectButton"
QT_MOC_LITERAL(4, 55, 13), // "sltHandleMenu"
QT_MOC_LITERAL(5, 69, 8), // "QAction*"
QT_MOC_LITERAL(6, 78, 6), // "action"
QT_MOC_LITERAL(7, 85, 22), // "sltDestroyEditorWidget"
QT_MOC_LITERAL(8, 108, 3) // "obj"

    },
    "ObjectElementEditor\0sltEditButton\0\0"
    "sltViewObjectButton\0sltHandleMenu\0"
    "QAction*\0action\0sltDestroyEditorWidget\0"
    "obj"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ObjectElementEditor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       4,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   34,    2, 0x0a /* Public */,
       3,    0,   35,    2, 0x0a /* Public */,
       4,    1,   36,    2, 0x0a /* Public */,
       7,    1,   39,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5,    6,
    QMetaType::Void, QMetaType::QObjectStar,    8,

       0        // eod
};

void ObjectElementEditor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ObjectElementEditor *_t = static_cast<ObjectElementEditor *>(_o);
        switch (_id) {
        case 0: _t->sltEditButton(); break;
        case 1: _t->sltViewObjectButton(); break;
        case 2: _t->sltHandleMenu((*reinterpret_cast< QAction*(*)>(_a[1]))); break;
        case 3: _t->sltDestroyEditorWidget((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject ObjectElementEditor::staticMetaObject = {
    { &ElementEditor::staticMetaObject, qt_meta_stringdata_ObjectElementEditor.data,
      qt_meta_data_ObjectElementEditor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ObjectElementEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ObjectElementEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ObjectElementEditor.stringdata))
        return static_cast<void*>(const_cast< ObjectElementEditor*>(this));
    return ElementEditor::qt_metacast(_clname);
}

int ObjectElementEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ElementEditor::qt_metacall(_c, _id, _a);
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
QT_END_MOC_NAMESPACE
