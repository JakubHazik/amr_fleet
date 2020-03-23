//amr_gui
// Created by jakub on 22. 3. 2020.
//

#include "../../include/amr_gui/monitoring/MonitorWidget.h"
#include <ui_MonitorWidget.h>


namespace amr_gui {

    MonitorWidget::MonitorWidget(QWidget *parent)
        :   QWidget(parent),
            ui(new Ui::MonitorWidget){

        // qt setup
        ui->setupUi(this);
        qRegisterMetaType<amr_msgs::ClientInfo >("ClientInfo");
        connect(this, SIGNAL(updateClientSignal(amr_msgs::ClientInfo)), this, SLOT(updateClientSlot(amr_msgs::ClientInfo)));

        // ros setup
        ros::NodeHandle nh("/");
        clientsInfoSub = nh.subscribe("/client_info", 5, &MonitorWidget::clientInfoCb, this);
    }

    void MonitorWidget::clientInfoCb(const amr_msgs::ClientInfo::Ptr& clientInfoMsg) {
        emit updateClientSignal(*clientInfoMsg);
    }

    void MonitorWidget::updateClientSlot(amr_msgs::ClientInfo clientInfo) {
        auto clientWidget = clientMonitorWidgets.find(clientInfo.clientId);
        if (clientWidget == clientMonitorWidgets.end()) {
            // create new tab widget
            auto* clientMonitorWidget = new ClientMonitorWidget;
            clientMonitorWidget->setObjectName(clientInfo.clientId.c_str());
            ui->tabWidget->addTab(clientMonitorWidget, clientInfo.clientId.c_str());

            // insert new tab widget to map
            std::pair<std::string, amr_gui::ClientMonitorWidget*> pair(clientInfo.clientId, clientMonitorWidget);
            auto insertResult = clientMonitorWidgets.insert(pair);
            clientWidget = insertResult.first;
        }

        clientWidget->second->updateClientInfo(clientInfo);
    }


}   // amr_gui namespace

//PLUGINLIB_EXPORT_CLASS(amr_gui::MonitorWidget, rqt_gui_cpp::Plugin)