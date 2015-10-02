#ifndef RVIZ_OVERLAYS_TEXT_H_
#define RVIZ_OVERLAYS_TEXT_H_

#include <QColor>
#include <QFont>
#include <QImage>
#include <QPen>
#include <QPainter>
#include <QRectF>
#include <QString>

namespace rviz_overlays {
namespace text {

inline void draw_text(QImage* canvas, const QRectF& bounding_box, const QString& text, const QFont& font, const QColor& color, int qt_alignment_flags = Qt::AlignLeft | Qt::AlignTop) {
  QPainter painter(canvas);
  painter.setFont(font);
  painter.setPen(QPen(color));
  painter.drawText(bounding_box, qt_alignment_flags, text);
  painter.end();
}

inline void centered(QImage* canvas, const QRectF& bounding_box, const QString& text, const QFont& font, const QColor& color) {
  draw_text(canvas, bounding_box, text, font, color, Qt::AlignHCenter | Qt::AlignTop);
}
}
}

#endif
