/****************************************************************************
** Meta object code from reading C++ file 'VariableArrayElementEditor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../VariableArrayElementEditor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'VariableArrayElementEditor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_VariableArrayElementEditor_t {
    QByteArrayData data[11];
    char stringdata[126];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_VariableArrayElementEditor_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_VariableArrayElementEditor_t qt_meta_stringdata_VariableArrayElementEditor = {
    {
QT_MOC_LITERAL(0, 0, 26), // "VariableArrayElementEditor"
QT_MOC_LITERAL(1, 27, 6), // "sltAdd"
QT_MOC_LITERAL(2, 34, 0), // ""
QT_MOC_LITERAL(3, 35, 9), // "sltDelete"
QT_MOC_LITERAL(4, 45, 16), // "sltItemActivated"
QT_MOC_LITERAL(5, 62, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(6, 79, 4), // "item"
QT_MOC_LITERAL(7, 84, 6), // "column"
QT_MOC_LITERAL(8, 91, 11), // "sltModified"
QT_MOC_LITERAL(9, 103, 18), // "sltEditorDestroyed"
QT_MOC_LITERAL(10, 122, 3) // "obj"

    },
    "VariableArrayElementEditor\0sltAdd\0\0"
    "sltDelete\0sltItemActivated\0QTreeWidgetItem*\0"
    "item\0column\0sltModified\0sltEditorDestroyed\0"
    "obj"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_VariableArrayElementEditor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       5,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   39,    2, 0x0a /* Public */,
       3,    0,   40,    2, 0x0a /* Public */,
       4,    2,   41,    2, 0x0a /* Public */,
       8,    0,   46,    2, 0x0a /* Public */,
       9,    1,   47,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 5, QMetaType::Int,    6,    7,
    QMetaType::Void,
    QMetaType::Void, QMetaType::QObjectStar,   10,

       0        // eod
};

void VariableArrayElementEditor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        VariableArrayElementEditor *_t = static_cast<VariableArrayElementEditor *>(_o);
        switch (_id) {
        case 0: _t->sltAdd(); break;
        case 1: _t->sltDelete(); break;
        case 2: _t->sltItemActivated((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 3: _t->sltModified(); break;
        case 4: _t->sltEditorDestroyed((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject VariableArrayElementEditor::staticMetaObject = {
    { &ElementEditor::staticMetaObject, qt_meta_stringdata_VariableArrayElementEditor.data,
      qt_meta_data_VariableArrayElementEditor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *VariableArrayElementEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *VariableArrayElementEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_VariableArrayElementEditor.stringdata))
        return static_cast<void*>(const_cast< VariableArrayElementEditor*>(this));
    return ElementEditor::qt_metacast(_clname);
}

int VariableArrayElementEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ElementEditor::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 5)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 5;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 5)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 5;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
