/********************************************************************************
** Form generated from reading UI file 'RobotSim.ui'
**
** Created: Sat Nov 12 21:49:46 2011
**      by: Qt User Interface Compiler version 4.7.4
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_ROBOTSIM_H
#define UI_ROBOTSIM_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDockWidget>
#include <QtGui/QGridLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QSplitter>
#include <QtGui/QStatusBar>
#include <QtGui/QTabWidget>
#include <QtGui/QTextBrowser>
#include <QtGui/QTextEdit>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_kauthamMain
{
public:
    QAction *actionNew;
    QAction *actionHelp;
    QAction *actionAbout;
    QWidget *centralwidget;
    QGridLayout *gridLayout_2;
    QSplitter *splitter;
    QTabWidget *viewsTab;
    QWidget *introTab;
    QGridLayout *gridLayout;
    QTextBrowser *textBrowser;
    QMenuBar *menubar;
    QMenu *menuActions;
    QMenu *menuFile;
    QMenu *menuHelp;
    QStatusBar *statusbar;
    QToolBar *toolBar;
    QDockWidget *outputWindow;
    QWidget *dockWidgetContents;
    QGridLayout *gridLayout1;
    QTextEdit *textEdit;

    void setupUi(QMainWindow *kauthamMain)
    {
        if (kauthamMain->objectName().isEmpty())
            kauthamMain->setObjectName(QString::fromUtf8("kauthamMain"));
        kauthamMain->setWindowModality(Qt::ApplicationModal);
        kauthamMain->resize(850, 759);
        QSizePolicy sizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(kauthamMain->sizePolicy().hasHeightForWidth());
        kauthamMain->setSizePolicy(sizePolicy);
        kauthamMain->setMinimumSize(QSize(850, 600));
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/icons/kautham.xpm"), QSize(), QIcon::Normal, QIcon::Off);
        kauthamMain->setWindowIcon(icon);
        kauthamMain->setToolButtonStyle(Qt::ToolButtonIconOnly);
        kauthamMain->setDockNestingEnabled(false);
        actionNew = new QAction(kauthamMain);
        actionNew->setObjectName(QString::fromUtf8("actionNew"));
        actionHelp = new QAction(kauthamMain);
        actionHelp->setObjectName(QString::fromUtf8("actionHelp"));
        actionAbout = new QAction(kauthamMain);
        actionAbout->setObjectName(QString::fromUtf8("actionAbout"));
        centralwidget = new QWidget(kauthamMain);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        gridLayout_2 = new QGridLayout(centralwidget);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        splitter = new QSplitter(centralwidget);
        splitter->setObjectName(QString::fromUtf8("splitter"));
        splitter->setMinimumSize(QSize(600, 500));
        splitter->setOrientation(Qt::Horizontal);
        viewsTab = new QTabWidget(splitter);
        viewsTab->setObjectName(QString::fromUtf8("viewsTab"));
        viewsTab->setEnabled(true);
        viewsTab->setMinimumSize(QSize(600, 4));
        introTab = new QWidget();
        introTab->setObjectName(QString::fromUtf8("introTab"));
        introTab->setEnabled(true);
        QSizePolicy sizePolicy1(QSizePolicy::Maximum, QSizePolicy::Maximum);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(introTab->sizePolicy().hasHeightForWidth());
        introTab->setSizePolicy(sizePolicy1);
        gridLayout = new QGridLayout(introTab);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        textBrowser = new QTextBrowser(introTab);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setSource(QUrl(""));
        textBrowser->setOpenExternalLinks(true);

        gridLayout->addWidget(textBrowser, 0, 0, 1, 1);

        viewsTab->addTab(introTab, QString());
        splitter->addWidget(viewsTab);

        gridLayout_2->addWidget(splitter, 0, 0, 1, 1);

        kauthamMain->setCentralWidget(centralwidget);
        menubar = new QMenuBar(kauthamMain);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 850, 25));
        menuActions = new QMenu(menubar);
        menuActions->setObjectName(QString::fromUtf8("menuActions"));
        menuFile = new QMenu(menubar);
        menuFile->setObjectName(QString::fromUtf8("menuFile"));
        menuHelp = new QMenu(menubar);
        menuHelp->setObjectName(QString::fromUtf8("menuHelp"));
        kauthamMain->setMenuBar(menubar);
        statusbar = new QStatusBar(kauthamMain);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        kauthamMain->setStatusBar(statusbar);
        toolBar = new QToolBar(kauthamMain);
        toolBar->setObjectName(QString::fromUtf8("toolBar"));
        toolBar->setOrientation(Qt::Horizontal);
        kauthamMain->addToolBar(Qt::TopToolBarArea, toolBar);
        outputWindow = new QDockWidget(kauthamMain);
        outputWindow->setObjectName(QString::fromUtf8("outputWindow"));
        QSizePolicy sizePolicy2(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(outputWindow->sizePolicy().hasHeightForWidth());
        outputWindow->setSizePolicy(sizePolicy2);
        outputWindow->setMinimumSize(QSize(125, 69));
        outputWindow->setBaseSize(QSize(122, 69));
        outputWindow->setAutoFillBackground(false);
        outputWindow->setFloating(false);
        outputWindow->setFeatures(QDockWidget::DockWidgetFloatable|QDockWidget::DockWidgetMovable|QDockWidget::DockWidgetVerticalTitleBar);
        outputWindow->setAllowedAreas(Qt::AllDockWidgetAreas);
        dockWidgetContents = new QWidget();
        dockWidgetContents->setObjectName(QString::fromUtf8("dockWidgetContents"));
        dockWidgetContents->setMinimumSize(QSize(100, 69));
        gridLayout1 = new QGridLayout(dockWidgetContents);
        gridLayout1->setObjectName(QString::fromUtf8("gridLayout1"));
        textEdit = new QTextEdit(dockWidgetContents);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        sizePolicy2.setHeightForWidth(textEdit->sizePolicy().hasHeightForWidth());
        textEdit->setSizePolicy(sizePolicy2);
        textEdit->setMinimumSize(QSize(50, 50));
        textEdit->setMaximumSize(QSize(16777215, 16777215));

        gridLayout1->addWidget(textEdit, 0, 0, 1, 1);

        outputWindow->setWidget(dockWidgetContents);
        kauthamMain->addDockWidget(static_cast<Qt::DockWidgetArea>(8), outputWindow);

        menubar->addAction(menuFile->menuAction());
        menubar->addAction(menuActions->menuAction());
        menubar->addAction(menuHelp->menuAction());
        menuHelp->addAction(actionHelp);
        menuHelp->addSeparator();
        menuHelp->addAction(actionAbout);

        retranslateUi(kauthamMain);

        viewsTab->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(kauthamMain);
    } // setupUi

    void retranslateUi(QMainWindow *kauthamMain)
    {
        kauthamMain->setWindowTitle(QApplication::translate("kauthamMain", "Kautham 2.0 - Institute of Industrial and Control Engineering - Technical University of Catalonia", 0, QApplication::UnicodeUTF8));
        actionNew->setText(QApplication::translate("kauthamMain", "New...", 0, QApplication::UnicodeUTF8));
        actionHelp->setText(QApplication::translate("kauthamMain", "Kautham Planner", 0, QApplication::UnicodeUTF8));
        actionAbout->setText(QApplication::translate("kauthamMain", "About...", 0, QApplication::UnicodeUTF8));
        textBrowser->setHtml(QApplication::translate("kauthamMain", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Ubuntu'; font-size:11pt; font-weight:400; font-style:normal;\">\n"
"<p style=\"-qt-paragraph-type:empty; margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; font-family:'MS Shell Dlg 2'; font-size:8pt;\"></p></body></html>", 0, QApplication::UnicodeUTF8));
        viewsTab->setTabText(viewsTab->indexOf(introTab), QApplication::translate("kauthamMain", "Introduction", 0, QApplication::UnicodeUTF8));
        menuActions->setTitle(QApplication::translate("kauthamMain", "Actions", 0, QApplication::UnicodeUTF8));
        menuFile->setTitle(QApplication::translate("kauthamMain", "File", 0, QApplication::UnicodeUTF8));
        menuHelp->setTitle(QApplication::translate("kauthamMain", "Help", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        outputWindow->setToolTip(QApplication::translate("kauthamMain", "Output Window", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        outputWindow->setWindowTitle(QApplication::translate("kauthamMain", "Output", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class kauthamMain: public Ui_kauthamMain {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_ROBOTSIM_H
