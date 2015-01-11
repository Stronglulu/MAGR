/****************************************************************************
** Meta object code from reading C++ file 'ObjectListElementEditor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../ObjectListElementEditor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'ObjectListElementEditor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_ObjectListElementEditor_t {
    QByteArrayData data[18];
    char stringdata[220];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_ObjectListElementEditor_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_ObjectListElementEditor_t qt_meta_stringdata_ObjectListElementEditor = {
    {
QT_MOC_LITERAL(0, 0, 23), // "ObjectListElementEditor"
QT_MOC_LITERAL(1, 24, 12), // "sltMoveToTop"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 15), // "sltMoveToBottom"
QT_MOC_LITERAL(4, 54, 9), // "sltMoveUp"
QT_MOC_LITERAL(5, 64, 11), // "sltMoveDown"
QT_MOC_LITERAL(6, 76, 7), // "sltEdit"
QT_MOC_LITERAL(7, 84, 13), // "sltEditMember"
QT_MOC_LITERAL(8, 98, 9), // "sltDelete"
QT_MOC_LITERAL(9, 108, 6), // "sltAdd"
QT_MOC_LITERAL(10, 115, 10), // "sltAddAddr"
QT_MOC_LITERAL(11, 126, 20), // "sltItemDoubleClicked"
QT_MOC_LITERAL(12, 147, 16), // "QTreeWidgetItem*"
QT_MOC_LITERAL(13, 164, 4), // "item"
QT_MOC_LITERAL(14, 169, 6), // "column"
QT_MOC_LITERAL(15, 176, 18), // "sltEditorDestroyed"
QT_MOC_LITERAL(16, 195, 3), // "obj"
QT_MOC_LITERAL(17, 199, 20) // "sltSubObjectModified"

    },
    "ObjectListElementEditor\0sltMoveToTop\0"
    "\0sltMoveToBottom\0sltMoveUp\0sltMoveDown\0"
    "sltEdit\0sltEditMember\0sltDelete\0sltAdd\0"
    "sltAddAddr\0sltItemDoubleClicked\0"
    "QTreeWidgetItem*\0item\0column\0"
    "sltEditorDestroyed\0obj\0sltSubObjectModified"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_ObjectListElementEditor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x0a /* Public */,
       3,    0,   75,    2, 0x0a /* Public */,
       4,    0,   76,    2, 0x0a /* Public */,
       5,    0,   77,    2, 0x0a /* Public */,
       6,    0,   78,    2, 0x0a /* Public */,
       7,    0,   79,    2, 0x0a /* Public */,
       8,    0,   80,    2, 0x0a /* Public */,
       9,    0,   81,    2, 0x0a /* Public */,
      10,    0,   82,    2, 0x0a /* Public */,
      11,    2,   83,    2, 0x0a /* Public */,
      15,    1,   88,    2, 0x0a /* Public */,
      17,    0,   91,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void, 0x80000000 | 12, QMetaType::Int,   13,   14,
    QMetaType::Void, QMetaType::QObjectStar,   16,
    QMetaType::Void,

       0        // eod
};

void ObjectListElementEditor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        ObjectListElementEditor *_t = static_cast<ObjectListElementEditor *>(_o);
        switch (_id) {
        case 0: _t->sltMoveToTop(); break;
        case 1: _t->sltMoveToBottom(); break;
        case 2: _t->sltMoveUp(); break;
        case 3: _t->sltMoveDown(); break;
        case 4: _t->sltEdit(); break;
        case 5: _t->sltEditMember(); break;
        case 6: _t->sltDelete(); break;
        case 7: _t->sltAdd(); break;
        case 8: _t->sltAddAddr(); break;
        case 9: _t->sltItemDoubleClicked((*reinterpret_cast< QTreeWidgetItem*(*)>(_a[1])),(*reinterpret_cast< int(*)>(_a[2]))); break;
        case 10: _t->sltEditorDestroyed((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        case 11: _t->sltSubObjectModified(); break;
        default: ;
        }
    }
}

const QMetaObject ObjectListElementEditor::staticMetaObject = {
    { &ElementEditor::staticMetaObject, qt_meta_stringdata_ObjectListElementEditor.data,
      qt_meta_data_ObjectListElementEditor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *ObjectListElementEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *ObjectListElementEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_ObjectListElementEditor.stringdata))
        return static_cast<void*>(const_cast< ObjectListElementEditor*>(this));
    return ElementEditor::qt_metacast(_clname);
}

int ObjectListElementEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ElementEditor::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
