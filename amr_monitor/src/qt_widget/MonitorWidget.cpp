//amr_gui
// Created by jakub on 22. 3. 2020.
//

#include <amr_monitor/qt_widget/MonitorWidget.h>
#include <ui_MonitorWidget.h>




namespace amr_gui {

    MonitorWidget::MonitorWidget(QWidget *parent)
        :   QWidget(parent),
            ui(new Ui::MonitorWidget){

        ui->setupUi(this);
        qRegisterMetaType<amr_msgs::ClientInfo >("ClientInfo");

        ros::NodeHandle nh("/");
        clientsInfoSub = nh.subscribe("/client_info", 5, &MonitorWidget::clientInfoCb, this);

        connect(this, SIGNAL(updateClientSignal(amr_msgs::ClientInfo)), this, SLOT(updateClientSlot(amr_msgs::ClientInfo)));
    }

    void MonitorWidget::clientInfoCb(const amr_msgs::ClientInfo::Ptr& clientInfoMsg) {
//        std::lock_guard<std::mutex> lock(clientInfoMtx);
        ROS_INFO("Client msg received");
        emit updateClientSignal(*clientInfoMsg);
    }

    void MonitorWidget::updateWidget() {
        for (const auto &msg: clientMonitorWidgets) {


//            ui->poseX->setNum(msg.second->poseWithCovariance.pose.pose.position.x);
//            ui->poseY->setNum(msg.second->poseWithCovariance.pose.pose.position.y);
        }
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