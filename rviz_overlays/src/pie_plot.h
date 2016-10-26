#ifndef RVIZ_OVERLAYS_PIE_PLOT_H_
#define RVIZ_OVERLAYS_PIE_PLOT_H_

#include <rviz/display.h>

#include "overlay_utils.h"

namespace rviz_overlays {
  class PiePlot : public rviz::Display {
    Q_OBJECT

  public:
    PiePlot();
    virtual ~PiePlot();

  protected:
    void draw();
    virtual void update(float wall_dt, float ros_dt);
    virtual void onInitialize();
    virtual void onEnable();
    virtual void onDisable();
    OverlayObject::Ptr overlay_;

  protected Q_SLOTS:
    // Qt-slots for UI Signals

  private:
  };

}

#endif
