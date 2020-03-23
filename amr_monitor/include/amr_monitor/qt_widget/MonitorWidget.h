//
// Created by jakub on 22. 3. 2020.
//

#ifndef SRC_MONITORWIDGET_H
#define SRC_MONITORWIDGET_H

#include <ros/ros.h>
#include <amr_msgs/ClientInfo.h>
#include <amr_monitor/qt_widget/ClientMonitorWidget.h>

#include <QWidget>
#include <QMetaType>

Q_DECLARE_METATYPE(amr_msgs::ClientInfo);

namespace Ui {
    class MonitorWidget;
}

namespace amr_gui{
    class MonitorWidget : public QWidget {
        Q_OBJECT

    public:
        explicit MonitorWidget(QWidget *parent = nullptr);
    private Q_SLOTS:
        void updateClientSlot(amr_msgs::ClientInfo clientInfo);

    Q_SIGNALS:
        void updateClientSignal(amr_msgs::ClientInfo clientInfo);

    private:
        Ui::MonitorWidget *ui;

        ros::Subscriber clientsInfoSub;
        std::map<std::string, amr_gui::ClientMonitorWidget*> clientMonitorWidgets;
        void clientInfoCb(const amr_msgs::ClientInfo::Ptr& clientInfoMsg);
    };

}
#endif //SRC_MONITORWIDGET_H
