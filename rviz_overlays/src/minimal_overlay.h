#ifndef RVIZ_OVERLAYS_MINIMAL_OVERLAY_H_
#define RVIZ_OVERLAYS_MINIMAL_OVERLAY_H_

#include <rviz/display.h>

#include "overlay_utils.h"

namespace rviz_overlays {
  class MinimalOverlay : public rviz::Display {
    Q_OBJECT

  public:
    MinimalOverlay();
    virtual ~MinimalOverlay();

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
