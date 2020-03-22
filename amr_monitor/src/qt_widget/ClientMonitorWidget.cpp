//
// Created by jakub on 22. 3. 2020.
//

#include <amr_monitor/qt_widget/ClientMonitorWidget.h>
#include <ui_ClientMonitorWidget.h>


namespace amr_gui {

    ClientMonitorWidget::ClientMonitorWidget(QWidget *parent)
            : QWidget(parent),
              ui(new Ui::ClientMonitorWidget) {

        ui->setupUi(this);


    }


    void ClientMonitorWidget::updateClientInfo(const amr_msgs::ClientInfo& clientInfo) {

    }

}