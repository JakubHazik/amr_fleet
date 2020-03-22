//
// Created by jakub on 22. 3. 2020.
//

#include <QtGui>
#include <QApplication>
#include <amr_gui/MainWindow.h>

int main(int argc, char **argv) {

    QApplication app(argc, argv);
    MainWindow w(argc, argv);
    w.show();

    return app.exec();
}