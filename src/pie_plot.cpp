#include "pie_plot.h"
#include "plotters.h"

#include <QPainter>
#include <QString>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
 #include <rviz/render_panel.h>

namespace rviz_overlays
{

  PiePlot::PiePlot()
    : rviz::Display() {
  }

  PiePlot::~PiePlot() {
    if (overlay_->isVisible()) {
      overlay_->hide();
    }
  }

  void PiePlot::onInitialize() {
    overlay_.reset(new OverlayObject());
    onEnable();
  }

  void PiePlot::update(float wall_dt, float ros_dt) {
    rviz::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
    int width = panel->width();
    int height = panel->height();
    overlay_->updateTextureSize(width, height);
    overlay_->setPosition(0, 0);
    overlay_->setDimensions(overlay_->getTextureWidth(),
                            overlay_->getTextureHeight());
    draw();
  }

  void PiePlot::draw() {
    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage hud = buffer.getQImage(*overlay_, Qt::transparent);

    QColor red(Qt::red);
    red.setAlpha(128);
    QColor green(Qt::green);
    green.setAlpha(64);
    int width = overlay_->getTextureWidth();
    int height = overlay_->getTextureHeight();
    int size = (height < width) ? height : width;
    plotters::pie(&hud, QRectF(0, 0, size*0.333, size*0.333), 0.333, red);
    plotters::pie(&hud, QRectF(size*0.333, 0, size*0.667, size*0.667), 0.667, green);
  }

  void PiePlot::onEnable() {
    overlay_->show();
  }

  void PiePlot::onDisable() {
    overlay_->hide();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_overlays::PiePlot, rviz::Display )
