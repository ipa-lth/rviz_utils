#include "minimal_overlay.h"

#include <QPainter>
#include <QString>
#include <rviz/display_context.h>
#include <rviz/view_manager.h>
#include <rviz/render_panel.h>

namespace rviz_overlays
{

  MinimalOverlay::MinimalOverlay()
    : rviz::Display() {
  }

  MinimalOverlay::~MinimalOverlay() {
    if (overlay_->isVisible()) {
      overlay_->hide();
    }
  }

  void MinimalOverlay::onInitialize() {
    overlay_.reset(new OverlayObject());
    onEnable();
  }

  void MinimalOverlay::update(float wall_dt, float ros_dt) {
    rviz::RenderPanel* panel = context_->getViewManager()->getRenderPanel();
    int width = panel->width();
    int height = panel->height();
    overlay_->updateTextureSize(width, height);
    overlay_->setPosition(0, 0);
    overlay_->setDimensions(overlay_->getTextureWidth(),
                            overlay_->getTextureHeight());
    draw();
  }

  void MinimalOverlay::draw() {
    // Get ready for Qt style painting
    ScopedPixelBuffer buffer = overlay_->getBuffer();
    QImage Hud = buffer.getQImage(*overlay_, Qt::transparent);
    // Prepare painter
    QPainter painter(&Hud);

    painter.setRenderHint(QPainter::Antialiasing, true);
    // Color, stroke width and style
    QColor red(Qt::red);
    red.setAlpha(128);
    int stroke = 6;
    painter.setPen(QPen(red, stroke, Qt::SolidLine));
    // Font
    QFont font("Helvetica");
    font.setPointSize(48);
    font.setBold(false);
    painter.setFont(font);
    // Draw across the whole overlay
    int width = overlay_->getTextureWidth();
    int height = overlay_->getTextureHeight();
    // Draw an ellipse
    painter.drawEllipse(stroke/2, stroke/2, width - stroke, height - stroke);
    // Draw some text
    QString s("Sample Text");
    painter.drawText(0, 0, width, height, Qt::AlignCenter | Qt::AlignVCenter, s);

    // done
    painter.end();
    // Unlock the pixel buffer
  }

  void MinimalOverlay::onEnable() {
    overlay_->show();
  }

  void MinimalOverlay::onDisable() {
    overlay_->hide();
  }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( rviz_overlays::MinimalOverlay, rviz::Display )
