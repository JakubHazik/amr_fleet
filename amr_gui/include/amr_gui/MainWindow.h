//
// Created by jakub on 22. 3. 2020.
//

#ifndef SRC_MAINWINDOW_H
#define SRC_MAINWINDOW_H

#include <QMainWindow>
#include <QWidget>

namespace Ui {
    class MainWindowDesign;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    ~MainWindow();
protected:

private slots:

private:
    Ui::MainWindowDesign *ui;
};

#endif //SRC_MAINWINDOW_H
