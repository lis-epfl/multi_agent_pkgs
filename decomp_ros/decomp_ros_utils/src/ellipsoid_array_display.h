#include <OgreSceneNode.h>
#include <OgreSceneManager.h>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/frame_manager_iface.hpp>

#include <rviz_common/load_resource.hpp>

#include <decomp_ros_msgs/msg/ellipsoid_array.h>
#include <rviz_common/message_filter_display.hpp>
#include "ellipsoid_array_visual.h"

namespace decomp_rviz_plugins {

class EllipsoidArrayVisual;

class EllipsoidArrayDisplay
    : public rviz_common::MessageFilterDisplay<decomp_ros_msgs::msg::EllipsoidArray> {
  Q_OBJECT
public:
  EllipsoidArrayDisplay();
  ~EllipsoidArrayDisplay();

protected:
  void onInitialize();

  void reset();

private Q_SLOTS:
  void updateColorAndAlpha();

private:
  void processMessage(decomp_ros_msgs::msg::EllipsoidArray::ConstSharedPtr msg);

  std::shared_ptr<EllipsoidArrayVisual> visual_;

  rviz_common::properties::ColorProperty *color_property_;
  rviz_common::properties::FloatProperty *alpha_property_;
};
}
