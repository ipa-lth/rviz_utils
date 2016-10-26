#ifndef RVIZ_OVERLAYS_TEXT_H_
#define RVIZ_OVERLAYS_TEXT_H_

#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QImage>
#include <QList>
#include <QListIterator>
#include <QPainter>
#include <QRectF>
#include <QString>

namespace rviz_overlays {
namespace text {

inline void box(QImage* canvas, const QRectF& bounding_box, const QString& text, const QFont& font, const QColor& color, int qt_alignment_flags = Qt::AlignLeft | Qt::AlignTop) {
  QPainter painter(canvas);
  painter.setFont(font);
  painter.setPen(color);
  painter.drawText(bounding_box, qt_alignment_flags, text);
}

inline void history(QImage* canvas, QPointF position, const QList<QString>& text_history, const QFont& font, QColor color, float fading = 1.0) {
  QFontMetrics metrics(font);
  float spacing = metrics.lineSpacing();
  QPainter painter(canvas);
  painter.setFont(font);

  QListIterator<QString> i(text_history);
  i.toBack();
  while(i.hasPrevious()) {
    painter.setPen(color);
    painter.drawText(position, i.previous());

    position.ry() += spacing;
    color.setAlphaF(color.alphaF() * fading);
  }
}

}
}

#endif
