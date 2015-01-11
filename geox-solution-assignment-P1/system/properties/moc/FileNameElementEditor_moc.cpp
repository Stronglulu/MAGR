/****************************************************************************
** Meta object code from reading C++ file 'FileNameElementEditor.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.4.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../FileNameElementEditor.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'FileNameElementEditor.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.4.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_FileNameElementEditor_t {
    QByteArrayData data[6];
    char stringdata[80];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_FileNameElementEditor_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_FileNameElementEditor_t qt_meta_stringdata_FileNameElementEditor = {
    {
QT_MOC_LITERAL(0, 0, 21), // "FileNameElementEditor"
QT_MOC_LITERAL(1, 22, 14), // "sltTextChanged"
QT_MOC_LITERAL(2, 37, 0), // ""
QT_MOC_LITERAL(3, 38, 4), // "text"
QT_MOC_LITERAL(4, 43, 16), // "sltReturnPressed"
QT_MOC_LITERAL(5, 60, 19) // "sltDotDotDotClicked"

    },
    "FileNameElementEditor\0sltTextChanged\0"
    "\0text\0sltReturnPressed\0sltDotDotDotClicked"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_FileNameElementEditor[] = {

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
       4,    0,   32,    2, 0x0a /* Public */,
       5,    0,   33,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void, QMetaType::QString,    3,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void FileNameElementEditor::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        FileNameElementEditor *_t = static_cast<FileNameElementEditor *>(_o);
        switch (_id) {
        case 0: _t->sltTextChanged((*reinterpret_cast< const QString(*)>(_a[1]))); break;
        case 1: _t->sltReturnPressed(); break;
        case 2: _t->sltDotDotDotClicked(); break;
        default: ;
        }
    }
}

const QMetaObject FileNameElementEditor::staticMetaObject = {
    { &ElementEditor::staticMetaObject, qt_meta_stringdata_FileNameElementEditor.data,
      qt_meta_data_FileNameElementEditor,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *FileNameElementEditor::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *FileNameElementEditor::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_FileNameElementEditor.stringdata))
        return static_cast<void*>(const_cast< FileNameElementEditor*>(this));
    return ElementEditor::qt_metacast(_clname);
}

int FileNameElementEditor::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = ElementEditor::qt_metacall(_c, _id, _a);
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
