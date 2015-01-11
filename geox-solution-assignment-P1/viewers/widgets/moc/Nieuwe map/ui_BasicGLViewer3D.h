/********************************************************************************
** Form generated from reading UI file 'BasicGLViewer3D.ui'
**
** Created by: Qt User Interface Compiler version 5.4.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_BASICGLVIEWER3D_H
#define UI_BASICGLVIEWER3D_H

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
#include "basicgeoxglwidget3d.h"

QT_BEGIN_NAMESPACE

class Ui_GLGeometryViewer3DImproved
{
public:
    QHBoxLayout *hboxLayout;
    QFrame *frame_2;
    QVBoxLayout *vboxLayout;
    QToolButton *btnResetCamera;
    QToolButton *btnDrawAxes;
    QToolButton *btnOrtho;
    QToolButton *btnYAxis;
    QSpacerItem *spacerItem;
    BasicGeoXGLWidget3D *glFrame;

    void setupUi(QWidget *GLGeometryViewer3DImproved)
    {
        if (GLGeometryViewer3DImproved->objectName().isEmpty())
            GLGeometryViewer3DImproved->setObjectName(QStringLiteral("GLGeometryViewer3DImproved"));
        GLGeometryViewer3DImproved->resize(782, 531);
        GLGeometryViewer3DImproved->setAutoFillBackground(true);
        hboxLayout = new QHBoxLayout(GLGeometryViewer3DImproved);
        hboxLayout->setSpacing(0);
        hboxLayout->setContentsMargins(0, 0, 0, 0);
        hboxLayout->setObjectName(QStringLiteral("hboxLayout"));
        frame_2 = new QFrame(GLGeometryViewer3DImproved);
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
        btnResetCamera = new QToolButton(frame_2);
        btnResetCamera->setObjectName(QStringLiteral("btnResetCamera"));
        btnResetCamera->setMinimumSize(QSize(46, 58));
        QIcon icon;
        icon.addFile(QStringLiteral(":/images/Camera.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnResetCamera->setIcon(icon);
        btnResetCamera->setIconSize(QSize(32, 32));
        btnResetCamera->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnResetCamera);

        btnDrawAxes = new QToolButton(frame_2);
        btnDrawAxes->setObjectName(QStringLiteral("btnDrawAxes"));
        btnDrawAxes->setMinimumSize(QSize(46, 58));
        btnDrawAxes->setCheckable(true);
        btnDrawAxes->setChecked(true);
        btnDrawAxes->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnDrawAxes);

        btnOrtho = new QToolButton(frame_2);
        btnOrtho->setObjectName(QStringLiteral("btnOrtho"));
        btnOrtho->setMinimumSize(QSize(46, 58));
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/images/wirecube.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnOrtho->setIcon(icon1);
        btnOrtho->setCheckable(true);
        btnOrtho->setChecked(false);
        btnOrtho->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnOrtho);

        btnYAxis = new QToolButton(frame_2);
        btnYAxis->setObjectName(QStringLiteral("btnYAxis"));
        btnYAxis->setMinimumSize(QSize(46, 58));
        btnYAxis->setCheckable(true);
        btnYAxis->setChecked(false);
        btnYAxis->setToolButtonStyle(Qt::ToolButtonTextUnderIcon);

        vboxLayout->addWidget(btnYAxis);

        spacerItem = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        vboxLayout->addItem(spacerItem);


        hboxLayout->addWidget(frame_2);

        glFrame = new BasicGeoXGLWidget3D(GLGeometryViewer3DImproved);
        glFrame->setObjectName(QStringLiteral("glFrame"));
        QSizePolicy sizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(glFrame->sizePolicy().hasHeightForWidth());
        glFrame->setSizePolicy(sizePolicy);
        glFrame->setMinimumSize(QSize(0, 0));
        glFrame->setMouseTracking(false);

        hboxLayout->addWidget(glFrame);


        retranslateUi(GLGeometryViewer3DImproved);

        QMetaObject::connectSlotsByName(GLGeometryViewer3DImproved);
    } // setupUi

    void retranslateUi(QWidget *GLGeometryViewer3DImproved)
    {
        GLGeometryViewer3DImproved->setWindowTitle(QApplication::translate("GLGeometryViewer3DImproved", "Form", 0));
        btnResetCamera->setText(QApplication::translate("GLGeometryViewer3DImproved", "Reset", 0));
        btnDrawAxes->setText(QApplication::translate("GLGeometryViewer3DImproved", "Show\n"
"Axes", 0));
        btnOrtho->setText(QApplication::translate("GLGeometryViewer3DImproved", "Orhto", 0));
        btnYAxis->setText(QApplication::translate("GLGeometryViewer3DImproved", "Rot.\n"
"Y-Axis", 0));
    } // retranslateUi

};

namespace Ui {
    class GLGeometryViewer3DImproved: public Ui_GLGeometryViewer3DImproved {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_BASICGLVIEWER3D_H
