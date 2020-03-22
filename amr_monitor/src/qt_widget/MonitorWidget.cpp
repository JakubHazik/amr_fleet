//
// Created by jakub on 22. 3. 2020.
//

#include <amr_monitor/qt_widget/MonitorWidget.h>
#include <ui_MonitorWidget.h>

//#include <ros/ros.h>


namespace amr_gui {

    MonitorWidget::MonitorWidget(QWidget *parent)
        :   QWidget(parent),
            ui(new Ui::MonitorWidget){

//            ROS_INFO("adasd");
        ui->setupUi(this);
    }


}

//PLUGINLIB_EXPORT_CLASS(amr_gui::MonitorWidget, rqt_gui_cpp::Plugin)