/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created: Mon Mar 20 12:31:07 2023
**      by: Qt User Interface Compiler version 4.8.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QGroupBox>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QMainWindow>
#include <QtGui/QMenu>
#include <QtGui/QMenuBar>
#include <QtGui/QStatusBar>
#include <QtGui/QToolBar>
#include <QtGui/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QAction *action_TaxiPlan;
    QAction *action_RandomSequence;
    QAction *action_Clear;
    QAction *action_taxitime;
    QAction *action_taxidistance;
    QAction *action_prioritizedplanning;
    QAction *action_ConflictDetect;
    QAction *actionStaticPath;
    QAction *actionBaseline;
    QAction *actionLabelSetting;
    QAction *actionGenerateFile;
    QAction *actionRobustPlan;
    QAction *actionMultipath;
    QAction *actionCompare;
    QAction *actionSPP;
    QAction *actionOriginalSPP;
    QAction *actionExpCompare;
    QAction *actionExpCmpWithoutHC;
    QAction *actionExpCmpFullHC;
    QAction *actionTest;
    QAction *actionExpLaneHC;
    QAction *actionTest_noMTT;
    QAction *actionTest_homogeneous;
    QAction *actionTest_homo_onestep;
    QAction *actionExp_WaitingTimeReduce;
    QAction *actionGenerate_runway_txt;
    QAction *actionGenerate_matrix_dist_txt;
    QAction *actionExp_revision;
    QAction *actionExp_revision_unimpeded;
    QWidget *centralWidget;
    QGroupBox *groupBox_set;
    QLineEdit *lineEdit_fitnessalias;
    QLabel *label_4;
    QLabel *label;
    QLineEdit *lineEdit_crossover;
    QLineEdit *lineEdit_mutation;
    QLineEdit *lineEdit_generation;
    QLabel *label_3;
    QLabel *label_2;
    QMenuBar *menuBar;
    QMenu *menu;
    QMenu *menuStaticSP;
    QMenu *menuComparison;
    QMenu *menuConvert;
    QMenu *menuRobustPlanning;
    QMenu *menuSPPNowaiting;
    QMenu *menuExtended;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(872, 308);
        QIcon icon;
        icon.addFile(QString::fromUtf8(":/resource/resource/583236.png"), QSize(), QIcon::Normal, QIcon::Off);
        MainWindow->setWindowIcon(icon);
        action_TaxiPlan = new QAction(MainWindow);
        action_TaxiPlan->setObjectName(QString::fromUtf8("action_TaxiPlan"));
        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/resource/resource/540814.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_TaxiPlan->setIcon(icon1);
        action_RandomSequence = new QAction(MainWindow);
        action_RandomSequence->setObjectName(QString::fromUtf8("action_RandomSequence"));
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/resource/resource/1126902.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_RandomSequence->setIcon(icon2);
        action_Clear = new QAction(MainWindow);
        action_Clear->setObjectName(QString::fromUtf8("action_Clear"));
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/resource/resource/43338.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_Clear->setIcon(icon3);
        action_taxitime = new QAction(MainWindow);
        action_taxitime->setObjectName(QString::fromUtf8("action_taxitime"));
        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/resource/resource/568815.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_taxitime->setIcon(icon4);
        action_taxidistance = new QAction(MainWindow);
        action_taxidistance->setObjectName(QString::fromUtf8("action_taxidistance"));
        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/resource/resource/14667.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_taxidistance->setIcon(icon5);
        action_prioritizedplanning = new QAction(MainWindow);
        action_prioritizedplanning->setObjectName(QString::fromUtf8("action_prioritizedplanning"));
        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/resource/resource/1124743.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_prioritizedplanning->setIcon(icon6);
        action_ConflictDetect = new QAction(MainWindow);
        action_ConflictDetect->setObjectName(QString::fromUtf8("action_ConflictDetect"));
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/resource/resource/13153.png"), QSize(), QIcon::Normal, QIcon::Off);
        action_ConflictDetect->setIcon(icon7);
        actionStaticPath = new QAction(MainWindow);
        actionStaticPath->setObjectName(QString::fromUtf8("actionStaticPath"));
        actionBaseline = new QAction(MainWindow);
        actionBaseline->setObjectName(QString::fromUtf8("actionBaseline"));
        actionLabelSetting = new QAction(MainWindow);
        actionLabelSetting->setObjectName(QString::fromUtf8("actionLabelSetting"));
        actionGenerateFile = new QAction(MainWindow);
        actionGenerateFile->setObjectName(QString::fromUtf8("actionGenerateFile"));
        actionRobustPlan = new QAction(MainWindow);
        actionRobustPlan->setObjectName(QString::fromUtf8("actionRobustPlan"));
        actionMultipath = new QAction(MainWindow);
        actionMultipath->setObjectName(QString::fromUtf8("actionMultipath"));
        actionCompare = new QAction(MainWindow);
        actionCompare->setObjectName(QString::fromUtf8("actionCompare"));
        actionSPP = new QAction(MainWindow);
        actionSPP->setObjectName(QString::fromUtf8("actionSPP"));
        actionOriginalSPP = new QAction(MainWindow);
        actionOriginalSPP->setObjectName(QString::fromUtf8("actionOriginalSPP"));
        actionExpCompare = new QAction(MainWindow);
        actionExpCompare->setObjectName(QString::fromUtf8("actionExpCompare"));
        actionExpCmpWithoutHC = new QAction(MainWindow);
        actionExpCmpWithoutHC->setObjectName(QString::fromUtf8("actionExpCmpWithoutHC"));
        actionExpCmpFullHC = new QAction(MainWindow);
        actionExpCmpFullHC->setObjectName(QString::fromUtf8("actionExpCmpFullHC"));
        actionTest = new QAction(MainWindow);
        actionTest->setObjectName(QString::fromUtf8("actionTest"));
        actionExpLaneHC = new QAction(MainWindow);
        actionExpLaneHC->setObjectName(QString::fromUtf8("actionExpLaneHC"));
        actionTest_noMTT = new QAction(MainWindow);
        actionTest_noMTT->setObjectName(QString::fromUtf8("actionTest_noMTT"));
        actionTest_homogeneous = new QAction(MainWindow);
        actionTest_homogeneous->setObjectName(QString::fromUtf8("actionTest_homogeneous"));
        actionTest_homo_onestep = new QAction(MainWindow);
        actionTest_homo_onestep->setObjectName(QString::fromUtf8("actionTest_homo_onestep"));
        actionExp_WaitingTimeReduce = new QAction(MainWindow);
        actionExp_WaitingTimeReduce->setObjectName(QString::fromUtf8("actionExp_WaitingTimeReduce"));
        actionGenerate_runway_txt = new QAction(MainWindow);
        actionGenerate_runway_txt->setObjectName(QString::fromUtf8("actionGenerate_runway_txt"));
        actionGenerate_matrix_dist_txt = new QAction(MainWindow);
        actionGenerate_matrix_dist_txt->setObjectName(QString::fromUtf8("actionGenerate_matrix_dist_txt"));
        actionExp_revision = new QAction(MainWindow);
        actionExp_revision->setObjectName(QString::fromUtf8("actionExp_revision"));
        actionExp_revision_unimpeded = new QAction(MainWindow);
        actionExp_revision_unimpeded->setObjectName(QString::fromUtf8("actionExp_revision_unimpeded"));
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        groupBox_set = new QGroupBox(centralWidget);
        groupBox_set->setObjectName(QString::fromUtf8("groupBox_set"));
        groupBox_set->setGeometry(QRect(61, 22, 491, 171));
        lineEdit_fitnessalias = new QLineEdit(groupBox_set);
        lineEdit_fitnessalias->setObjectName(QString::fromUtf8("lineEdit_fitnessalias"));
        lineEdit_fitnessalias->setGeometry(QRect(241, 64, 191, 20));
        label_4 = new QLabel(groupBox_set);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(71, 62, 121, 21));
        label = new QLabel(groupBox_set);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(71, 100, 131, 21));
        lineEdit_crossover = new QLineEdit(groupBox_set);
        lineEdit_crossover->setObjectName(QString::fromUtf8("lineEdit_crossover"));
        lineEdit_crossover->setGeometry(QRect(241, 102, 191, 20));
        lineEdit_mutation = new QLineEdit(groupBox_set);
        lineEdit_mutation->setObjectName(QString::fromUtf8("lineEdit_mutation"));
        lineEdit_mutation->setGeometry(QRect(241, 132, 191, 20));
        lineEdit_generation = new QLineEdit(groupBox_set);
        lineEdit_generation->setObjectName(QString::fromUtf8("lineEdit_generation"));
        lineEdit_generation->setGeometry(QRect(241, 32, 191, 20));
        label_3 = new QLabel(groupBox_set);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(71, 30, 101, 21));
        label_2 = new QLabel(groupBox_set);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(70, 130, 131, 21));
        MainWindow->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QString::fromUtf8("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 872, 26));
        menu = new QMenu(menuBar);
        menu->setObjectName(QString::fromUtf8("menu"));
        menuStaticSP = new QMenu(menuBar);
        menuStaticSP->setObjectName(QString::fromUtf8("menuStaticSP"));
        menuComparison = new QMenu(menuBar);
        menuComparison->setObjectName(QString::fromUtf8("menuComparison"));
        menuConvert = new QMenu(menuBar);
        menuConvert->setObjectName(QString::fromUtf8("menuConvert"));
        menuRobustPlanning = new QMenu(menuBar);
        menuRobustPlanning->setObjectName(QString::fromUtf8("menuRobustPlanning"));
        menuSPPNowaiting = new QMenu(menuBar);
        menuSPPNowaiting->setObjectName(QString::fromUtf8("menuSPPNowaiting"));
        menuExtended = new QMenu(menuBar);
        menuExtended->setObjectName(QString::fromUtf8("menuExtended"));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QString::fromUtf8("mainToolBar"));
        mainToolBar->setIconSize(QSize(20, 20));
        mainToolBar->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QString::fromUtf8("statusBar"));
        MainWindow->setStatusBar(statusBar);

        menuBar->addAction(menu->menuAction());
        menuBar->addAction(menuComparison->menuAction());
        menuBar->addAction(menuStaticSP->menuAction());
        menuBar->addAction(menuConvert->menuAction());
        menuBar->addAction(menuRobustPlanning->menuAction());
        menuBar->addAction(menuSPPNowaiting->menuAction());
        menuBar->addAction(menuExtended->menuAction());
        menu->addAction(action_TaxiPlan);
        menu->addAction(action_taxitime);
        menu->addAction(action_taxidistance);
        menu->addAction(action_ConflictDetect);
        menuStaticSP->addAction(actionStaticPath);
        menuComparison->addAction(actionBaseline);
        menuComparison->addAction(actionLabelSetting);
        menuConvert->addAction(actionGenerateFile);
        menuRobustPlanning->addAction(actionRobustPlan);
        menuRobustPlanning->addAction(actionMultipath);
        menuRobustPlanning->addAction(actionCompare);
        menuSPPNowaiting->addAction(actionOriginalSPP);
        menuSPPNowaiting->addAction(actionSPP);
        menuSPPNowaiting->addAction(actionExpCompare);
        menuSPPNowaiting->addAction(actionExpCmpWithoutHC);
        menuSPPNowaiting->addAction(actionExpCmpFullHC);
        menuSPPNowaiting->addAction(actionExpLaneHC);
        menuExtended->addAction(actionTest);
        menuExtended->addAction(actionTest_noMTT);
        menuExtended->addAction(actionTest_homogeneous);
        menuExtended->addAction(actionTest_homo_onestep);
        menuExtended->addAction(actionExp_WaitingTimeReduce);
        menuExtended->addAction(actionExp_revision_unimpeded);
        menuExtended->addAction(actionExp_revision);
        menuExtended->addSeparator();
        menuExtended->addAction(actionGenerate_runway_txt);
        menuExtended->addAction(actionGenerate_matrix_dist_txt);
        mainToolBar->addAction(action_prioritizedplanning);
        mainToolBar->addAction(action_Clear);
        mainToolBar->addAction(action_RandomSequence);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Exp. Env. for AGMO", 0, QApplication::UnicodeUTF8));
        action_TaxiPlan->setText(QApplication::translate("MainWindow", "TaxiPlan", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        action_TaxiPlan->setToolTip(QApplication::translate("MainWindow", "Sequential Planning", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        action_RandomSequence->setText(QApplication::translate("MainWindow", "RandomSequence", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        action_RandomSequence->setToolTip(QApplication::translate("MainWindow", "Generate Random Sequence", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        action_Clear->setText(QApplication::translate("MainWindow", "ClearAll", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        action_Clear->setToolTip(QApplication::translate("MainWindow", "Clear Temporal Data", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        action_taxitime->setText(QApplication::translate("MainWindow", "TaxiTime", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        action_taxitime->setToolTip(QApplication::translate("MainWindow", "Print Total Taxi Time", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        action_taxidistance->setText(QApplication::translate("MainWindow", "TaxiDistance", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        action_taxidistance->setToolTip(QApplication::translate("MainWindow", "Print Total Taxi Distance", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        action_prioritizedplanning->setText(QApplication::translate("MainWindow", "GA Planning", 0, QApplication::UnicodeUTF8));
        action_ConflictDetect->setText(QApplication::translate("MainWindow", "ConflictDetect", 0, QApplication::UnicodeUTF8));
        actionStaticPath->setText(QApplication::translate("MainWindow", "StaticPath", 0, QApplication::UnicodeUTF8));
        actionBaseline->setText(QApplication::translate("MainWindow", "SP_Baseline", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionBaseline->setToolTip(QApplication::translate("MainWindow", "Sequential planning upon the baseline model", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionLabelSetting->setText(QApplication::translate("MainWindow", "SP_LabelSetting", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionLabelSetting->setToolTip(QApplication::translate("MainWindow", "Sequential planning using Label-setting algorithm", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionGenerateFile->setText(QApplication::translate("MainWindow", "GenerateFile", 0, QApplication::UnicodeUTF8));
#ifndef QT_NO_TOOLTIP
        actionGenerateFile->setToolTip(QApplication::translate("MainWindow", "generate the node connectivity and distance matrix", 0, QApplication::UnicodeUTF8));
#endif // QT_NO_TOOLTIP
        actionRobustPlan->setText(QApplication::translate("MainWindow", "Singlpath", 0, QApplication::UnicodeUTF8));
        actionMultipath->setText(QApplication::translate("MainWindow", "Multipath", 0, QApplication::UnicodeUTF8));
        actionCompare->setText(QApplication::translate("MainWindow", "Compare", 0, QApplication::UnicodeUTF8));
        actionSPP->setText(QApplication::translate("MainWindow", "SPPTW-NH", 0, QApplication::UnicodeUTF8));
        actionOriginalSPP->setText(QApplication::translate("MainWindow", "DRT-NH", 0, QApplication::UnicodeUTF8));
        actionExpCompare->setText(QApplication::translate("MainWindow", "ExpCompare", 0, QApplication::UnicodeUTF8));
        actionExpCmpWithoutHC->setText(QApplication::translate("MainWindow", "ExpCmpWithoutHC", 0, QApplication::UnicodeUTF8));
        actionExpCmpFullHC->setText(QApplication::translate("MainWindow", "ExpCmpFullHC", 0, QApplication::UnicodeUTF8));
        actionTest->setText(QApplication::translate("MainWindow", "test", 0, QApplication::UnicodeUTF8));
        actionExpLaneHC->setText(QApplication::translate("MainWindow", "ExpLaneHC", 0, QApplication::UnicodeUTF8));
        actionTest_noMTT->setText(QApplication::translate("MainWindow", "test_noMTT", 0, QApplication::UnicodeUTF8));
        actionTest_homogeneous->setText(QApplication::translate("MainWindow", "test_homogeneous", 0, QApplication::UnicodeUTF8));
        actionTest_homo_onestep->setText(QApplication::translate("MainWindow", "test_homo_onestep", 0, QApplication::UnicodeUTF8));
        actionExp_WaitingTimeReduce->setText(QApplication::translate("MainWindow", "Exp_WaitingTimeReduce", 0, QApplication::UnicodeUTF8));
        actionGenerate_runway_txt->setText(QApplication::translate("MainWindow", "generate'runway.txt'", 0, QApplication::UnicodeUTF8));
        actionGenerate_matrix_dist_txt->setText(QApplication::translate("MainWindow", "generate'matrix_dist.txt'", 0, QApplication::UnicodeUTF8));
        actionExp_revision->setText(QApplication::translate("MainWindow", "Exp_revision", 0, QApplication::UnicodeUTF8));
        actionExp_revision_unimpeded->setText(QApplication::translate("MainWindow", "Exp_revision_unimpeded", 0, QApplication::UnicodeUTF8));
        groupBox_set->setTitle(QString());
        label_4->setText(QApplication::translate("MainWindow", "Fitness alias:", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("MainWindow", "Crossover rate:", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("MainWindow", "Generation:", 0, QApplication::UnicodeUTF8));
        label_2->setText(QApplication::translate("MainWindow", "Mutation rate:", 0, QApplication::UnicodeUTF8));
        menu->setTitle(QApplication::translate("MainWindow", "SequentialPlanning", 0, QApplication::UnicodeUTF8));
        menuStaticSP->setTitle(QApplication::translate("MainWindow", "StaticShortestPath", 0, QApplication::UnicodeUTF8));
        menuComparison->setTitle(QApplication::translate("MainWindow", "Comparison", 0, QApplication::UnicodeUTF8));
        menuConvert->setTitle(QApplication::translate("MainWindow", "Convert", 0, QApplication::UnicodeUTF8));
        menuRobustPlanning->setTitle(QApplication::translate("MainWindow", "RobustPlanning", 0, QApplication::UnicodeUTF8));
        menuSPPNowaiting->setTitle(QApplication::translate("MainWindow", "Nowaiting", 0, QApplication::UnicodeUTF8));
        menuExtended->setTitle(QApplication::translate("MainWindow", "Extended", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
