//
// Created by jakub on 24. 3. 2020.
//

#ifndef SRC_CONTROLWIDGET_H
#define SRC_CONTROLWIDGET_H

#include <ros/ros.h>
#include <amr_gui/control/ControlInterface.h>

#include <QWidget>

namespace Ui {
    class ControlWidget;
}

namespace amr_gui{
    class ControlWidget : public QWidget {
        Q_OBJECT

    public:
        explicit ControlWidget(QWidget *parent = nullptr);
    private Q_SLOTS:
        void on_applyCommandBtn_clicked();
        void on_reachabilitySetBtn_clicked();
        void on_startTeleopBtn_clicked();
        void on_stopTeleopBtn_clicked();
//
//        Q_SIGNALS:
//                void updateClientSignal(amr_msgs::ClientInfo clientInfo);

    private:
        Ui::ControlWidget *ui;

        ControlInterface controlInterface;


    };

}

#endif //SRC_CONTROLWIDGET_H
