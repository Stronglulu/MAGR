/********************************************************************************
** Form generated from reading UI file 'GeoXMainWindowDesigner.ui'
**
** Created by: Qt User Interface Compiler version 5.4.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GEOXMAINWINDOWDESIGNER_H
#define UI_GEOXMAINWINDOWDESIGNER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "ClassTypeClassPropertyEditorWidget.h"

QT_BEGIN_NAMESPACE

class Ui_GeoXMainWindow
{
public:
    QAction *actionOpen;
    QAction *actionSaveAs;
    QAction *actionExit;
    QWidget *centralwidget;
    QHBoxLayout *hboxLayout;
    QFrame *mainFrame;
    QMenuBar *menubar;
    QMenu *menu_File;
    QToolBar *toolBar;
    QDockWidget *dockObjectView;
    QWidget *dockWidgetContents_2;
    QVBoxLayout *vboxLayout;
    QVBoxLayout *vboxLayout1;
    QLabel *label;
    ClassTypeClassPropertyEditorWidget *experimentSelector;
    QFrame *experimentObjectViewFrame;
    QDockWidget *dockOutput;
    QWidget *dockWidgetContents;
    QHBoxLayout *hboxLayout1;
    QTextBrowser *outputWindow;

    void setupUi(QMainWindow *GeoXMainWindow)
    {
        if (GeoXMainWindow->objectName().isEmpty())
            GeoXMainWindow->setObjectName(QStringLiteral("GeoXMainWindow"));
        GeoXMainWindow->resize(855, 699);
        GeoXMainWindow->setMinimumSize(QSize(300, 200));
        GeoXMainWindow->setFocusPolicy(Qt::StrongFocus);
        GeoXMainWindow->setContextMenuPolicy(Qt::DefaultContextMenu);
        GeoXMainWindow->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        GeoXMainWindow->setAnimated(false);
        actionOpen = new QAction(GeoXMainWindow);
        actionOpen->setObjectName(QStringLiteral("actionOpen"));
        const QIcon icon = QIcon(QString::fromUtf8(":/images/open.png"));
        actionOpen->setIcon(icon);
        actionSaveAs = new QAction(GeoXMainWindow);
        actionSaveAs->setObjectName(QStringLiteral("actionSaveAs"));
        const QIcon icon1 = QIcon(QString::fromUtf8(":/images/saveHS.png"));
        actionSaveAs->setIcon(icon1);
        actionExit = new QAction(GeoXMainWindow);
        actionExit->setObjectName(QStringLiteral("actionExit"));
        const QIcon icon2 = QIcon(QString::fromUtf8(":/images/exit.png"));
        actionExit->setIcon(icon2);
        centralwidget = new QWidget(GeoXMainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        hboxLayout = new QHBoxLayout(centralwidget);
        hboxLayout->setSpacing(0);
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        mainFrame = new QFrame(centralwidget);
        mainFrame->setObjectName(QStringLiteral("mainFrame"));
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(7), static_cast<QSizePolicy::Policy>(7));
        sizePolicy.setHorizontalStretch(1);
        sizePolicy.setVerticalStretch(1);
        sizePolicy.setHeightForWidth(mainFrame->sizePolicy().hasHeightForWidth());
        mainFrame->setSizePolicy(sizePolicy);
        mainFrame->setAutoFillBackground(false);
        mainFrame->setFrameShape(QFrame::NoFrame);
        mainFrame->setFrameShadow(QFrame::Plain);

        hboxLayout->addWidget(mainFrame);

        GeoXMainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(GeoXMainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 855, 22));
        menubar->setFocusPolicy(Qt::StrongFocus);
        menu_File = new QMenu(menubar);
        menu_File->setObjectName(QStringLiteral("menu_File"));
        GeoXMainWindow->setMenuBar(menubar);
        toolBar = new QToolBar(GeoXMainWindow);
        toolBar->setObjectName(QStringLiteral("toolBar"));
        toolBar->setOrientation(Qt::Horizontal);
        GeoXMainWindow->addToolBar(static_cast<Qt::ToolBarArea>(4), toolBar);
        dockObjectView = new QDockWidget(GeoXMainWindow);
        dockObjectView->setObjectName(QStringLiteral("dockObjectView"));
        dockObjectView->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable|QDockWidget::NoDockWidgetFeatures);
        dockObjectView->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents_2 = new QWidget();
        dockWidgetContents_2->setObjectName(QStringLiteral("dockWidgetContents_2"));
        vboxLayout = new QVBoxLayout(dockWidgetContents_2);
        vboxLayout->setSpacing(3);
        vboxLayout->setContentsMargins(0, 0, 0, 0);
        vboxLayout->setObjectName(QStringLiteral("vboxLayout"));
        vboxLayout1 = new QVBoxLayout();
        vboxLayout1->setSpacing(3);
        vboxLayout1->setContentsMargins(0, 0, 0, 0);
        vboxLayout1->setObjectName(QStringLiteral("vboxLayout1"));
        label = new QLabel(dockWidgetContents_2);
        label->setObjectName(QStringLiteral("label"));
        QFont font;
        font.setBold(true);
        font.setWeight(75);
        label->setFont(font);

        vboxLayout1->addWidget(label);

        experimentSelector = new ClassTypeClassPropertyEditorWidget(dockWidgetContents_2);
        experimentSelector->setObjectName(QStringLiteral("experimentSelector"));

        vboxLayout1->addWidget(experimentSelector);


        vboxLayout->addLayout(vboxLayout1);

        experimentObjectViewFrame = new QFrame(dockWidgetContents_2);
        experimentObjectViewFrame->setObjectName(QStringLiteral("experimentObjectViewFrame"));
        QSizePolicy sizePolicy1(static_cast<QSizePolicy::Policy>(5), static_cast<QSizePolicy::Policy>(7));
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(experimentObjectViewFrame->sizePolicy().hasHeightForWidth());
        experimentObjectViewFrame->setSizePolicy(sizePolicy1);
        experimentObjectViewFrame->setMinimumSize(QSize(250, 16));
        experimentObjectViewFrame->setFrameShape(QFrame::NoFrame);
        experimentObjectViewFrame->setFrameShadow(QFrame::Raised);

        vboxLayout->addWidget(experimentObjectViewFrame);

        dockObjectView->setWidget(dockWidgetContents_2);
        GeoXMainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(2), dockObjectView);
        dockOutput = new QDockWidget(GeoXMainWindow);
        dockOutput->setObjectName(QStringLiteral("dockOutput"));
        dockOutput->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable|QDockWidget::NoDockWidgetFeatures);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QStringLiteral("dockWidgetContents"));
        hboxLayout1 = new QHBoxLayout(dockWidgetContents);
        hboxLayout1->setSpacing(0);
        hboxLayout1->setContentsMargins(0, 0, 0, 0);
        hboxLayout1->setObjectName(QStringLiteral("hboxLayout1"));
        outputWindow = new QTextBrowser(dockWidgetContents);
        outputWindow->setObjectName(QStringLiteral("outputWindow"));

        hboxLayout1->addWidget(outputWindow);

        dockOutput->setWidget(dockWidgetContents);
        GeoXMainWindow->addDockWidget(static_cast<Qt::DockWidgetArea>(8), dockOutput);

        menubar->addAction(menu_File->menuAction());
        menu_File->addAction(actionOpen);
        menu_File->addAction(actionSaveAs);
        menu_File->addSeparator();
        menu_File->addAction(actionExit);
        toolBar->addAction(actionOpen);
        toolBar->addAction(actionSaveAs);
        toolBar->addSeparator();
        toolBar->addAction(actionExit);

        retranslateUi(GeoXMainWindow);

        QMetaObject::connectSlotsByName(GeoXMainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *GeoXMainWindow)
    {
        GeoXMainWindow->setWindowTitle(QApplication::translate("GeoXMainWindow", "MainWindow", 0));
        actionOpen->setText(QApplication::translate("GeoXMainWindow", "&Open Experiment...", 0));
        actionOpen->setIconText(QApplication::translate("GeoXMainWindow", "Open...", 0));
#ifndef QT_NO_TOOLTIP
        actionOpen->setToolTip(QApplication::translate("GeoXMainWindow", "Open Experiment...", 0));
#endif // QT_NO_TOOLTIP
        actionOpen->setShortcut(QApplication::translate("GeoXMainWindow", "Ctrl+O", 0));
        actionSaveAs->setText(QApplication::translate("GeoXMainWindow", "Save Experiment &As...", 0));
        actionSaveAs->setIconText(QApplication::translate("GeoXMainWindow", "Save As...", 0));
#ifndef QT_NO_TOOLTIP
        actionSaveAs->setToolTip(QApplication::translate("GeoXMainWindow", "Save Experiment As...", 0));
#endif // QT_NO_TOOLTIP
        actionSaveAs->setShortcut(QApplication::translate("GeoXMainWindow", "Ctrl+S", 0));
        actionExit->setText(QApplication::translate("GeoXMainWindow", "E&xit", 0));
        actionExit->setIconText(QApplication::translate("GeoXMainWindow", "Exit...", 0));
#ifndef QT_NO_TOOLTIP
        actionExit->setToolTip(QApplication::translate("GeoXMainWindow", "Exit...", 0));
#endif // QT_NO_TOOLTIP
        actionExit->setShortcut(QApplication::translate("GeoXMainWindow", "Alt+X", 0));
        menu_File->setTitle(QApplication::translate("GeoXMainWindow", "&File", 0));
        toolBar->setWindowTitle(QApplication::translate("GeoXMainWindow", "toolBar", 0));
        dockObjectView->setWindowTitle(QApplication::translate("GeoXMainWindow", "Object View", 0));
        label->setText(QApplication::translate("GeoXMainWindow", "Experiment:", 0));
        dockOutput->setWindowTitle(QApplication::translate("GeoXMainWindow", "Output", 0));
    } // retranslateUi

};

namespace Ui {
    class GeoXMainWindow: public Ui_GeoXMainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GEOXMAINWINDOWDESIGNER_H
