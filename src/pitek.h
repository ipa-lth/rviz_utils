#ifndef RVIZ_OVERLAYS_PITEK_H_
#define RVIZ_OVERLAYS_PITEK_H_

#include <QImage>

#include <geometry_msgs/Vector3.h>
#include <ros/ros.h>
#include <rviz/display.h>
#include <rviz/properties/bool_property.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/properties/ros_topic_property.h>

#include "overlay_utils.h"

namespace rviz_overlays {
  class Pitek : public rviz::Display {
    Q_OBJECT

  public:
    Pitek();
    virtual ~Pitek();

  protected:
    void draw();
    virtual void update(float wall_dt, float ros_dt);
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    void recordForces(geometry_msgs::Vector3::ConstPtr msg);
    void truncateForces();
    void truncateForce(QVector<QPointF>* force);


    OverlayObject::Ptr overlay_;
    QImage* logo_;

    ros::Subscriber sub_forces_;
    //ros::Subscriber sub_state_progress;
    //ros::Subscriber sub_state_name;

    // State
    // Forces
    QVector<QPointF>* force_x_;
    QVector<QPointF>* force_y_;
    QVector<QPointF>* force_z_;

    // Config
    // Top level
    rviz::BoolProperty* p_forces_;
    rviz::BoolProperty* p_logo_;
    rviz::BoolProperty* p_state_;
    rviz::FloatProperty* p_alpha_;
    rviz::IntProperty* p_margin_;
    // State
    rviz::ColorProperty* p_state_color_;
    rviz::IntProperty* p_state_pie_size_;
    rviz::IntProperty* p_state_font_size_;
    rviz::StringProperty* p_state_font_;
    //Forces
    rviz::FloatProperty* p_forces_time_;
    rviz::IntProperty* p_forces_w_;
    rviz::IntProperty* p_forces_h_;
    rviz::IntProperty* p_forces_padding_;
    rviz::RosTopicProperty* p_forces_topic_;
    // Logo
    rviz::FloatProperty* p_logo_scale_;


  protected Q_SLOTS:
    void pForcesTopicUpdate();

  private:
  };

}

#endif
