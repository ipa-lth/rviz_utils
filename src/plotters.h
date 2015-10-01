#ifndef RVIZ_OVERLAYS_MINIMAL_OVERLAY_H_
#define RVIZ_OVERLAYS_MINIMAL_OVERLAY_H_

#include <QColor>
#include <QImage>
#include <QPainter>
#include <QRectF>

namespace rviz_overlays {
namespace plotters {

QRectF margins_removed(const QRectF rect, float margin) {
  return rect.adjusted(margin, margin, -margin, -margin);
}

void pie(QImage* canvas, QRectF bounding_box, float normalized_value, QColor color) {
  const float factor = bounding_box.width();
  const float stroke_width = 0.04 * factor;
  const float value_line_width = 0.08 * factor;
  const float indicator_line_width = 0.01 * factor;
  const float value_margin = 0.08 * factor;

  const float outer_margin = stroke_width / 2;
  const float inner_margin = outer_margin + value_margin;

  QPainter painter(canvas);
  painter.setRenderHint(QPainter::Antialiasing, true);

  // Draw the stroke circle
  painter.setPen(QPen(color, stroke_width, Qt::SolidLine));
  int margin = stroke_width / 2;
  painter.drawEllipse(margins_removed(bounding_box, outer_margin));

  // Draw the value line
  const float start_angle = -90;
  const float value_angle = -normalized_value * 360.0;
  painter.setPen(QPen(color, value_line_width, Qt::SolidLine, Qt::FlatCap));
  painter.drawArc(margins_removed(bounding_box, inner_margin), start_angle * 16, value_angle * 16);

  // Draw the indicator line
  const float indicator_angle = (1 - normalized_value) * 360.0;
  painter.setPen(QPen(color, indicator_line_width, Qt::SolidLine, Qt::FlatCap));
  painter.drawArc(margins_removed(bounding_box, inner_margin), start_angle * 16, indicator_angle * 16);

  painter.end();
}
}
}

#endif
