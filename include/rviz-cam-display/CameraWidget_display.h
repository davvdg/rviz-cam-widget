#ifndef IMU_DISPLAY_H
#define IMU_DISPLAY_H
/*
#ifndef Q_MOC_RUN
#include <boost/circular_buffer.hpp>
#endif
*/
#include <sensor_msgs/CameraInfo.h>
#include <rviz/message_filter_display.h>
#include "rviz-cam-display/CameraWidget_visual.h"

namespace Ogre
{
class SceneNode;
}

namespace rviz
{
class ColorProperty;
class FloatProperty;
class IntProperty;
}

// All the source in this plugin is in its own namespace.  This is not
// required but is good practice.
namespace cam_display
{

class CamWidgetDisplay;


class CamWidgetDisplay: public rviz::MessageFilterDisplay< sensor_msgs::CameraInfo >
{
Q_OBJECT
public:
  // Constructor.  pluginlib::ClassLoader creates instances by calling
  // the default constructor, so make sure you have one.
  CamWidgetDisplay();
  virtual ~CamWidgetDisplay();

  // Overrides of protected virtual functions from Display.  As much
  // as possible, when Displays are not enabled, they should not be
  // subscribed to incoming data and should not show anything in the
  // 3D view.  These functions are where these connections are made
  // and broken.
protected:
  virtual void onInitialize();

  // A helper to clear this display back to the initial state.
  virtual void reset();


  // These Qt slots get connected to signals indicating changes in the user-editable properties.
private Q_SLOTS:
  void updateColorAndAlpha();
  void updateScale();
  void updateHistoryLength();
  void updateQueueSize();

  // Function to handle an incoming ROS message.
private:
  void processMessage( const sensor_msgs::CameraInfo::ConstPtr& msg );

  // Storage for the list of visuals.  It is a circular buffer where
  // data gets popped from the front (oldest) and pushed to the back (newest)
  // User-editable property variables.


  rviz::ColorProperty* color_property_;
  rviz::FloatProperty* alpha_property_;

  rviz::IntProperty* history_length_property_;
  rviz::IntProperty* queue_size_property_;

  rviz::FloatProperty* scale_property_;
  float scale_;

  std::deque<std::shared_ptr<CamWidgetVisual>> p_visual_history_;
  int visual_history_maxsize_;
  Ogre::ColourValue color_;
  //std::shared_ptr<CamWidgetVisual> p_visual_;
};
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

#endif // IMU_DISPLAY_H
// %EndTag(FULL_SOURCE)%