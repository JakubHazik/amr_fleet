//
// Created by jakub on 22. 3. 2020.
//

#include <amr_monitor/MonitorWidget.h>
//#include <rqt_gui_cpp/plugin.h>
#include <amr_monitor/ui_MonitorWidget.h>

#include <ros/ros.h>


namespace amr_gui {

    MonitorWidget::MonitorWidget(QWidget *parent) : QWidget(parent) {
            ROS_INFO("adasd");

    }


}

//PLUGINLIB_EXPORT_CLASS(amr_gui::MonitorWidget, rqt_gui_cpp::Plugin)