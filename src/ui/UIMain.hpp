/********************************************************************************
** Form generated from reading UI file 'UIMain.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UIMAIN_H
#define UIMAIN_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QListView>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_mainWindow
{
public:
    QWidget *centralwidget;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *verticalLayout_3;
    QListView *lstProjects;
    QHBoxLayout *horizontalLayout_2;
    QPushButton *pushButton_3;
    QPushButton *pushButton_4;
    QPushButton *pushButton_5;
    QGridLayout *gridLayout;
    QLabel *lblIm;
    QListView *lstIm;
    QVBoxLayout *verticalLayout_2;
    QPushButton *btnImagen;
    QPushButton *btnPointcloud;

    void setupUi(QMainWindow *mainWindow)
    {
        if (mainWindow->objectName().isEmpty())
            mainWindow->setObjectName(QStringLiteral("mainWindow"));
        mainWindow->resize(1735, 927);
        mainWindow->setMinimumSize(QSize(0, 0));
        mainWindow->setMaximumSize(QSize(5000, 5000));
        centralwidget = new QWidget(mainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        horizontalLayout = new QHBoxLayout(centralwidget);
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        lstProjects = new QListView(centralwidget);
        lstProjects->setObjectName(QStringLiteral("lstProjects"));

        verticalLayout_3->addWidget(lstProjects);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        horizontalLayout_2->setContentsMargins(-1, 10, -1, -1);
        pushButton_3 = new QPushButton(centralwidget);
        pushButton_3->setObjectName(QStringLiteral("pushButton_3"));

        horizontalLayout_2->addWidget(pushButton_3);

        pushButton_4 = new QPushButton(centralwidget);
        pushButton_4->setObjectName(QStringLiteral("pushButton_4"));

        horizontalLayout_2->addWidget(pushButton_4);

        pushButton_5 = new QPushButton(centralwidget);
        pushButton_5->setObjectName(QStringLiteral("pushButton_5"));

        horizontalLayout_2->addWidget(pushButton_5);


        verticalLayout_3->addLayout(horizontalLayout_2);


        horizontalLayout->addLayout(verticalLayout_3);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(-1, -1, 0, -1);
        lblIm = new QLabel(centralwidget);
        lblIm->setObjectName(QStringLiteral("lblIm"));
        QSizePolicy sizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(lblIm->sizePolicy().hasHeightForWidth());
        lblIm->setSizePolicy(sizePolicy);
        lblIm->setMinimumSize(QSize(1280, 720));
        lblIm->setMaximumSize(QSize(1280, 720));
        lblIm->setLayoutDirection(Qt::LeftToRight);

        gridLayout->addWidget(lblIm, 0, 0, 1, 1);

        lstIm = new QListView(centralwidget);
        lstIm->setObjectName(QStringLiteral("lstIm"));
        sizePolicy.setHeightForWidth(lstIm->sizePolicy().hasHeightForWidth());
        lstIm->setSizePolicy(sizePolicy);
        lstIm->setMinimumSize(QSize(1280, 150));
        lstIm->setMaximumSize(QSize(1280, 150));
        lstIm->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
        lstIm->setFlow(QListView::LeftToRight);

        gridLayout->addWidget(lstIm, 1, 0, 1, 1);


        horizontalLayout->addLayout(gridLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        verticalLayout_2->setContentsMargins(30, -1, 30, -1);
        btnImagen = new QPushButton(centralwidget);
        btnImagen->setObjectName(QStringLiteral("btnImagen"));
        btnImagen->setLayoutDirection(Qt::LeftToRight);
        QIcon icon;
        icon.addFile(QStringLiteral("iconos/guardar.png"), QSize(), QIcon::Normal, QIcon::Off);
        btnImagen->setIcon(icon);

        verticalLayout_2->addWidget(btnImagen);

        btnPointcloud = new QPushButton(centralwidget);
        btnPointcloud->setObjectName(QStringLiteral("btnPointcloud"));

        verticalLayout_2->addWidget(btnPointcloud);


        horizontalLayout->addLayout(verticalLayout_2);

        mainWindow->setCentralWidget(centralwidget);

        retranslateUi(mainWindow);

        QMetaObject::connectSlotsByName(mainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *mainWindow)
    {
        mainWindow->setWindowTitle(QApplication::translate("mainWindow", "Dfrag Smart", Q_NULLPTR));
        pushButton_3->setText(QApplication::translate("mainWindow", "PushButton", Q_NULLPTR));
        pushButton_4->setText(QApplication::translate("mainWindow", "PushButton", Q_NULLPTR));
        pushButton_5->setText(QApplication::translate("mainWindow", "PushButton", Q_NULLPTR));
        lblIm->setText(QString());
        btnImagen->setText(QString());
        btnPointcloud->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class mainWindow: public Ui_mainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UIMAIN_H
