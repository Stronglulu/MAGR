/****************************************************************************
** Meta object code from reading C++ file 'EmbeddedObjectElementEditor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../EmbeddedObjectElementEditor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'EmbeddedObjectElementEditor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_EmbeddedObjectElementEditor_t {
    QByteArrayData data[5];
    char stringdata[66];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_EmbeddedObjectElementEditor_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_EmbeddedObjectElementEditor_t qt_meta_stringdata_EmbeddedObjectElementEditor = {
    {
QT_MOC_LITERAL(0, 0, 27), // "EmbeddedObjectElementEditor"
QT_MOC_LITERAL(1, 28, 9), // "sltButton"
QT_MOC_LITERAL(2, 38, 0), // ""
QT_MOC_LITERAL(3, 39, 22), // "sltDestroyEditorWidget"
QT_MOC_LITERAL(4, 62, 3) // "obj"

    },
    "EmbeddedObjectElementEditor\0sltButton\0"
    "\0sltDestroyEditorWidget\0obj"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_EmbeddedObjectElementEditor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       2,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   24,    2, 0x0a /* Public */,
       3,    1,   25,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void, QMetaType::QObjectStar,    4,

       0        // eod
};

void EmbeddedObjectElementEditor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        EmbeddedObjectElementEditor *_t = static_cast<EmbeddedObjectElementEditor *>(_o);
        switch (_id) {
        case 0: _t->sltButton(); break;
        case 1: _t->sltDestroyEditorWidget((*reinterpret_cast< QObject*(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject EmbeddedObjectElementEditor::staticMetaObject = {
    { &ElementEditor::staticMetaObject, qt_meta_stringdata_EmbeddedObjectElementEditor.data,
      qt_meta_data_EmbeddedObjectElementEditor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *EmbeddedObjectElementEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *EmbeddedObjectElementEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_EmbeddedObjectElementEditor.stringdata))
        return static_cast<void*>(const_cast< EmbeddedObjectElementEditor*>(this));
    return ElementEditor::qt_metacast(_clname);
}

int EmbeddedObjectElementEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ElementEditor::qt_metacall(_c, _id, _a);
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
struct qt_meta_stringdata_EmbeddedObjectInlineElementEditor_t {
    QByteArrayData data[1];
    char stringdata[34];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_EmbeddedObjectInlineElementEditor_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_EmbeddedObjectInlineElementEditor_t qt_meta_stringdata_EmbeddedObjectInlineElementEditor = {
    {
QT_MOC_LITERAL(0, 0, 33) // "EmbeddedObjectInlineElementEd..."

    },
    "EmbeddedObjectInlineElementEditor"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_EmbeddedObjectInlineElementEditor[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       0,    0, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

       0        // eod
};

void EmbeddedObjectInlineElementEditor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    Q_UNUSED(_o);
    Q_UNUSED(_id);
    Q_UNUSED(_c);
    Q_UNUSED(_a);
}

const QMetaObject EmbeddedObjectInlineElementEditor::staticMetaObject = {
    { &ElementEditor::staticMetaObject, qt_meta_stringdata_EmbeddedObjectInlineElementEditor.data,
      qt_meta_data_EmbeddedObjectInlineElementEditor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *EmbeddedObjectInlineElementEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *EmbeddedObjectInlineElementEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_EmbeddedObjectInlineElementEditor.stringdata))
        return static_cast<void*>(const_cast< EmbeddedObjectInlineElementEditor*>(this));
    return ElementEditor::qt_metacast(_clname);
}

int EmbeddedObjectInlineElementEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ElementEditor::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    return _id;
}
QT_END_MOC_NAMESPACE
