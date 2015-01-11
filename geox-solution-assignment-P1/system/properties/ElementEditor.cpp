//---------------------------------------------------------------------------
#include "stdafx.h"
//---------------------------------------------------------------------------
#include "ElementEditor.h"
//---------------------------------------------------------------------------



ElementEditor::ElementEditor(QWidget * parent, Qt::WindowFlags f)
:QWidget( parent, f )
{
	m_Offset = NULL;
}

ElementEditor::~ElementEditor(void)
{
}

void ElementEditor::modified()
{
	emit sigModified();
}

void ElementEditor::sltUpdate()
{
	updateEditor();
}

#ifdef _WIN32
#include "moc/ELementEditor_moc.cpp"
#else
#include "moc_ElementEditor.cpp"
#endif
