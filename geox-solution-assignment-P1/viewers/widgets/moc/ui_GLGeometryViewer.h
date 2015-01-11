/********************************************************************************
** Form generated from reading UI file 'GLGeometryViewer.ui'
**
** Created by: Qt User Interface Compiler version 5.4.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_GLGEOMETRYVIEWER_H
#define UI_GLGEOMETRYVIEWER_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QToolButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "GeoXGLWidget.h"

QT_BEGIN_NAMESPACE

class Ui_GLGeometryViewer
{
public:
    QHBoxLayout *hboxLayout;
    QFrame *frame_2;
    QVBoxLayout *vboxLayout;
    QToolButton *btnCamera;
    QToolButton *btnPick;
    QToolButton *btnClear;
    QToolButton *btnCreate;
    QSpacerItem *spacerItem;
    GeoXGLWidget *glFrame;

    void setupUi(QWidget *GLGeometryViewer)
    {
        if (GLGeometryViewer->objectName().isEmpty())
            GLGeometryViewer->setObjectName(QStringLiteral("GLGeometryViewer"));
        GLGeometryViewer->resize(782, 531);
        GLGeometryViewer->setAutoFillBackground(true);
        hboxLayout = new QHBoxLayout(GLGeometryViewer);
        hboxLayout->setSpacing(0);
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        frame_2 = new QFrame(GLGeometryViewer);
        frame_2->setObjectName(QStringLiteral("frame_2"));
        frame_2->setMinimumSize(QSize(54, 16));
        frame_2->setMaximumSize(QSize(54, 16777215));
        frame_2->setFrameShape(QFrame::StyledPanel);
        frame_2->setFrameShadow(QFrame::Raised);
        vboxLayout = new QVBoxLayout(frame_2);
#ifndef Q_OS_MAC
        vboxLayout->setSpacing(6);
#endif
        vboxLayout->setContentsMargins(3, 3, 3, 3);
        vboxLayout->setObjectName(QStringLiteral("vboxLayout"));
        btnCamera = new QToolButton(frame_2);
        btnCamera->setObjectName(QStringLiteral("btnCamera"));
        btnCamera->setMinimumSize(QSize(46, 40));
        const QIcon icon = QIcon(QString::fromUtf8(":/images/Camera.png"));
        btnCamera->setIcon(icon);
        btnCamera->setIconSize(QSize(32, 32));
        btnCamera->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnCamera);

        btnPick = new QToolButton(frame_2);
        btnPick->setObjectName(QStringLiteral("btnPick"));
        btnPick->setMinimumSize(QSize(46, 40));
        btnPick->setSizeIncrement(QSize(0, 0));
        const QIcon icon1 = QIcon(QString::fromUtf8(":/images/SelectPoint.png"));
        btnPick->setIcon(icon1);
        btnPick->setIconSize(QSize(32, 32));
        btnPick->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnPick);

        btnClear = new QToolButton(frame_2);
        btnClear->setObjectName(QStringLiteral("btnClear"));
        btnClear->setMinimumSize(QSize(46, 40));
        const QIcon icon2 = QIcon(QString::fromUtf8(":/images/400px-Broom_icon.svg.png"));
        btnClear->setIcon(icon2);
        btnClear->setIconSize(QSize(32, 32));
        btnClear->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnClear);

        btnCreate = new QToolButton(frame_2);
        btnCreate->setObjectName(QStringLiteral("btnCreate"));
        btnCreate->setMinimumSize(QSize(46, 56));
        const QIcon icon3 = QIcon(QString::fromUtf8(":/images/100px-Nuvola_Green_Plus.svg.png"));
        btnCreate->setIcon(icon3);
        btnCreate->setIconSize(QSize(32, 32));
        btnCreate->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnCreate);

        spacerItem = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        vboxLayout->addItem(spacerItem);


        hboxLayout->addWidget(frame_2);

        glFrame = new GeoXGLWidget(GLGeometryViewer);
        glFrame->setObjectName(QStringLiteral("glFrame"));
        QSizePolicy sizePolicy(static_cast<QSizePolicy::Policy>(13), static_cast<QSizePolicy::Policy>(13));
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(glFrame->sizePolicy().hasHeightForWidth());
        glFrame->setSizePolicy(sizePolicy);
        glFrame->setMinimumSize(QSize(16, 16));
        glFrame->setMouseTracking(false);

        hboxLayout->addWidget(glFrame);


        retranslateUi(GLGeometryViewer);

        QMetaObject::connectSlotsByName(GLGeometryViewer);
    } // setupUi

    void retranslateUi(QWidget *GLGeometryViewer)
    {
        GLGeometryViewer->setWindowTitle(QApplication::translate("GLGeometryViewer", "Form", 0));
        btnCamera->setText(QApplication::translate("GLGeometryViewer", "camera", 0));
        btnPick->setText(QApplication::translate("GLGeometryViewer", "select", 0));
        btnClear->setText(QApplication::translate("GLGeometryViewer", "clear", 0));
        btnCreate->setText(QApplication::translate("GLGeometryViewer", "add", 0));
    } // retranslateUi

};

namespace Ui {
    class GLGeometryViewer: public Ui_GLGeometryViewer {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_GLGEOMETRYVIEWER_H
