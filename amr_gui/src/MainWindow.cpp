//
// Created by jakub on 22. 3. 2020.
//

#include <amr_gui/MainWindow.h>
#include <ui_MainWindow.h>

MainWindow::MainWindow(int argc, char **argv, QWidget *parent)
    :   QMainWindow(parent),
        ui(new Ui::MainWindowDesign){

    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
//    delete ui;
}