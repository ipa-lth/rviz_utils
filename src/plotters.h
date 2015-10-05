#ifndef RVIZ_OVERLAYS_PLOTTERS_H_
#define RVIZ_OVERLAYS_PLOTTERS_H_

#include <QColor>
#include <QImage>
#include <QPainter>
#include <QRectF>
#include <QTransform>

namespace rviz_overlays {
namespace plotters {

inline QRectF margins_removed(const QRectF rect, float margin) {
  return rect.adjusted(margin, margin, -margin, -margin);
}

inline void timeseries(QImage* canvas, QRectF bounding_box, const QVector<QPointF>* values, int width, QColor color, bool antialiasing) {
  // find min and max
  QPointF min(values->first().x(), std::numeric_limits<double>::max());
  QPointF max(values->last().x(), std::numeric_limits<double>::min());
  for(QVector<QPointF>::const_iterator i = values->constBegin(); i < values->constEnd(); ++i) {
    if(i->y() < min.y())
      min.setY(i->y());
    else if(i->y() > max.y())
      max.setY(i->y());
  }
  QPainter painter(canvas);
  painter.setRenderHint(QPainter::Antialiasing, antialiasing);
  QPen pen(color, width, Qt::SolidLine, Qt::FlatCap, Qt::BevelJoin);
  pen.setCosmetic(true); // width not affected by transform
  painter.setPen(pen);
  //painter.drawRect(bounding_box);

  // Figure out transform to fit points into box
  QPointF span = max - min;
  QTransform T;
  T.translate(bounding_box.left(), bounding_box.bottom());
  // Note: flipped y axis
  T.scale(bounding_box.width() / span.x(), -bounding_box.height() / span.y());
  T.translate(-min.x(), -min.y());
  painter.setTransform(T);
  // Slow operation below!
  painter.drawPolyline(values->constData(), values->size());
  painter.end();
}

inline void pie(QImage* canvas, QRectF bounding_box, float normalized_value, QColor color) {
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
