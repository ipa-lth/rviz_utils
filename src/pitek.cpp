#include "pitek.h"
#include "plotters.h"
#include "text.h"

#include <QPainter>
#include <QString>

#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>

namespace rviz_overlays
{

  Pitek::Pitek() : rviz::Display() {
    // Config
    // Top level
    p_margin_ = new rviz::IntProperty("Margin", 20, "description", this);
    p_alpha_ = new rviz::FloatProperty("Alpha", 1.0, "description", this);
    p_state_ = new rviz::BoolProperty("State", true, "description", this);
    p_forces_ = new rviz::BoolProperty("Forces", true, "description", this);
    p_logo_ = new rviz::BoolProperty("Logo", true, "description", this);
    // State
    p_state_color_ = new rviz::ColorProperty("Color", QColor(50, 100, 255, 128), "description", p_state_);
    p_state_pie_size_ = new rviz::IntProperty("Pie Size", 100, "description", p_state_);
    p_state_font_size_ = new rviz::IntProperty("Font Size", 48, "description", p_state_);
    p_state_name_topic_ = new rviz::RosTopicProperty("Name Topic", "",
      ros::message_traits::datatype<std_msgs::String>(), "description",
      p_state_, SLOT(pStateNameTopicUpdate()), this);
    p_state_progress_topic_ = new rviz::RosTopicProperty("Progress Topic", "",
      ros::message_traits::datatype<std_msgs::Float64>(), "description",
      p_state_, SLOT(pStateProgressTopicUpdate()), this);
    p_state_font_ = new rviz::StringProperty("Font", "Helvetica", "description", p_state_);
    //Forces
    p_forces_topic_ = new rviz::RosTopicProperty("Topic", "",
      ros::message_traits::datatype<geometry_msgs::Vector3>(), "description",
      p_forces_, SLOT(pForcesTopicUpdate()), this);
    p_forces_time_ = new rviz::FloatProperty("Time Window", 2.0, "description", p_forces_);
    p_forces_w_ = new rviz::IntProperty("Width", 300, "description", p_forces_);
    p_forces_h_ = new rviz::IntProperty("Height", 180, "description", p_forces_);
    p_forces_padding_ = new rviz::IntProperty("Padding", 50, "description", p_forces_);
    // Logo
    p_logo_scale_ = new rviz::FloatProperty("Scale", 0.6, "description", p_logo_);
  }

  Pitek::~Pitek() {

    if (overlay_->isVisible()) {
      overlay_->hide();
    }
  }

  void Pitek::onInitialize() {
    overlay_.reset(new OverlayObject());
    // find the logo
    QString path = QString::fromStdString(ros::package::getPath("rviz_overlays"));
    logo_ = new QImage(path+"/etc/logo.png");
    // State
    force_x_ = new QVector<QPointF>();
    force_y_ = new QVector<QPointF>();
    force_z_ = new QVector<QPointF>();
    // Subscribers
    pForcesTopicUpdate();
    pStateNameTopicUpdate();
    pStateProgressTopicUpdate();
    onEnable();
  }

  void Pitek::update(float wall_dt, float ros_dt) {
    // Find the current panel size
    rviz::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
    int width = panel->width();
    int height = panel->height();
    // Make the overlay cover everything
    overlay_->updateTextureSize(width, height);
    overlay_->setPosition(0, 0);
    overlay_->setDimensions(overlay_->getTextureWidth(),
                            overlay_->getTextureHeight());
    // Process state
    truncateForces();
    truncateStateNames(5); // Config?

    draw();
  }

  void Pitek::draw() {
    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage img = buffer.getQImage(*overlay_, Qt::transparent);
    int width = overlay_->getTextureWidth();
    int height = overlay_->getTextureHeight();
    int margin = p_margin_->getInt();
    QRectF viewport = plotters::margins_removed(QRectF(0, 0, width, height), margin);

    // States
    // TODO config
    float fading = 0.6;
    // States
    QColor c = p_state_color_->getColor();
    c.setAlphaF(p_alpha_->getFloat());
    int s = p_state_pie_size_->getInt();
    QFont font = QFont(p_state_font_->getString());
    font.setPixelSize(p_state_font_size_->getInt());
    QRectF pie_rect(viewport.topLeft(), QSizeF(s, s));
    plotters::pie(&img, pie_rect, state_progress_, c);
    QPointF state_text_point = pie_rect.bottomLeft() + 1.5 * QPointF(0, font.pixelSize());
    text::history(&img, state_text_point, state_names_, font, c, fading);

    // Forces
    // TODO config
    int line_width = 1;
    // Forces
    c = p_state_color_->getColor();
    // x
    QRectF plot_rect(0, 0, p_forces_w_->getInt(), p_forces_h_->getInt());
    plot_rect.moveTopRight(viewport.topRight());
    c.setRgbF(1, 0, 0);
    c.setAlphaF(p_alpha_->getFloat());
    plotters::timeseries(&img, plot_rect, force_x_, line_width, c);
    // y
    plot_rect.moveTop(plot_rect.top()+plot_rect.height()+p_forces_padding_->getInt());
    c.setRgbF(0, 1, 0);
    c.setAlphaF(p_alpha_->getFloat());
    plotters::timeseries(&img, plot_rect, force_y_, line_width, c);
    // z
    plot_rect.moveTop(plot_rect.top()+plot_rect.height()+p_forces_padding_->getInt());
    c.setRgbF(0, 0, 1);
    c.setAlphaF(p_alpha_->getFloat());
    plotters::timeseries(&img, plot_rect, force_z_, line_width, c);

    // Logo
    QPainter painter(&img);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    float sc = p_logo_scale_->getFloat();
    QRectF logo_rect(0, 0, sc * logo_->width(), sc * logo_->height());
    logo_rect.moveBottomRight(viewport.bottomRight());
    painter.drawImage(logo_rect, *logo_);
    painter.end();

  }

  void Pitek::pForcesTopicUpdate() {
    sub_forces_.shutdown();
    std::string topic_name = p_forces_topic_->getTopicStd();
    ros::NodeHandle n;
    sub_forces_ = n.subscribe(topic_name, 1, &Pitek::recordForces, this);
  }

  void Pitek::recordForces(geometry_msgs::Vector3::ConstPtr msg) {
    double t = ros::Time::now().toSec();
    force_x_->append(QPointF(t, msg->x));
    force_y_->append(QPointF(t, msg->y));
    force_z_->append(QPointF(t, msg->z));
  }

  void Pitek::truncateForces() {
    truncateForce(force_x_);
    truncateForce(force_y_);
    truncateForce(force_z_);
  }

  void Pitek::truncateForce(QVector<QPointF>* force) {
    if(force->isEmpty())
      return;

    double w = p_forces_time_->getFloat();
    double earliest = force->last().x() - w;

    QVector<QPointF>::iterator truncate = force->begin();
    while(truncate != force->end() && truncate->x() < earliest)
      truncate = force->erase(truncate);
  }

  void Pitek::pStateNameTopicUpdate() {
    sub_state_name_.shutdown();
    std::string topic_name = p_state_name_topic_->getTopicStd();
    ros::NodeHandle n;
    sub_state_name_ = n.subscribe(topic_name, 1, &Pitek::recordStateName, this);
  }

  void Pitek::pStateProgressTopicUpdate() {
    sub_state_progress_.shutdown();
    std::string topic_name = p_state_progress_topic_->getTopicStd();
    ros::NodeHandle n;
    sub_state_progress_ = n.subscribe(topic_name, 1, &Pitek::recordStateProgress, this);
  }

  void Pitek::recordStateName(std_msgs::String::ConstPtr msg) {
    state_names_.append(QString::fromStdString(msg->data));
  }

  void Pitek::truncateStateNames(unsigned int length) {
    int truncate = state_names_.length() - length;
    if(truncate > 0) {
      state_names_.erase(state_names_.begin(), state_names_.begin() + truncate);
    }
  }

  void Pitek::recordStateProgress(std_msgs::Float64::ConstPtr msg) {
    state_progress_ = msg->data;
  }

  void Pitek::onEnable() {
    overlay_->show();
  }

  void Pitek::onDisable() {
    overlay_->hide();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_overlays::Pitek, rviz::Display )
