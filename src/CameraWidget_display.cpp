
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <tf/transform_listener.h>

#include <rviz/visualization_manager.h>
#include <rviz/properties/color_property.h>
#include <rviz/properties/float_property.h>
#include <rviz/properties/int_property.h>
#include <rviz/frame_manager.h>

#include "rviz-cam-display/CameraWidget_visual.h"

#include "rviz-cam-display/CameraWidget_display.h"




namespace cam_display
{

// BEGIN_TUTORIAL
// The constructor must have no arguments, so we can't give the
// constructor the parameters it needs to fully initialize.
CamWidgetDisplay::CamWidgetDisplay()
{
  /*
  color_property_ = new rviz::ColorProperty( "Color", QColor( 204, 51, 204 ),
                                             "Color to draw the acceleration arrows.",
                                             this, SLOT( updateColorAndAlpha() ));

  alpha_property_ = new rviz::FloatProperty( "Alpha", 1.0,
                                             "0 is fully transparent, 1.0 is fully opaque.",
                                             this, SLOT( updateColorAndAlpha() ));

  history_length_property_ = new rviz::IntProperty( "History Length", 1,
                                                    "Number of prior measurements to display.",
                                                    this, SLOT( updateHistoryLength() ));
  history_length_property_->setMin( 1 );
  history_length_property_->setMax( 100000 );
  */
  return;
}

// After the top-level rviz::Display::initialize() does its own setup,
// it calls the subclass's onInitialize() function.  This is where we
// instantiate all the workings of the class.  We make sure to also
// call our immediate super-class's onInitialize() function, since it
// does important stuff setting up the message filter.
//
//  Note that "MFDClass" is a typedef of
// ``MessageFilterDisplay<message type>``, to save typing that long
// templated class name every time you need to refer to the
// superclass.
void CamWidgetDisplay::onInitialize()
{
  
  MFDClass::onInitialize();
  Ogre::Quaternion orientation( 0.0, 0.0, 0.0, 1.0);
  Ogre::Vector3 position(0.0, 0.0, 0.0);  
  p_visual_ = std::make_shared<CamWidgetVisual>( context_->getSceneManager(), scene_node_ );

  //visual.setColor( 1.0, 1.0, 1.0, 1.0 );
  //visual.setFrameOrientation( orientation );
  //visual.setFramePosition( position );

  /*
  updateHistoryLength();
  */

  return;
}

CamWidgetDisplay::~CamWidgetDisplay()
{
  return;
}

// Clear the visuals by deleting their objects.
void CamWidgetDisplay::reset()
{
  
  rviz::Display::reset();
  
  return;
}

// Set the current color and alpha values for each visual.
void CamWidgetDisplay::updateColorAndAlpha()
{
  /*
  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  */
  return;
}

// Set the number of past visuals to show.
void CamWidgetDisplay::updateHistoryLength()
{
  return;
}
void CamWidgetDisplay::processMessage( const sensor_msgs::CameraInfo::ConstPtr& msg ) {

  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  std::cout << msg->header.frame_id << std::endl;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }
  //p_visual_->setMessage( msg );
  p_visual_->setFramePosition( position );
  p_visual_->setFrameOrientation( orientation );

  float foc_x = msg->K[0] / ((float)(msg->width) * 2.0) ;
  float foc_y = msg->K[4] / ((float)(msg->height) * 2.0);

  p_visual_->setCameraFoc(foc_x, foc_y);

  return;
}
// This is our callback to handle an incoming message.
//void CamWidgetDisplay::processMessage( const sensor_msgs::Imu::ConstPtr& msg )
//{
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  /*
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if( !context_->getFrameManager()->getTransform( msg->header.frame_id,
                                                  msg->header.stamp,
                                                  position, orientation ))
  {
    ROS_DEBUG( "Error transforming from frame '%s' to frame '%s'",
               msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    return;
  }
  */
  // We are keeping a circular buffer of visual pointers.  This gets
  // the next one, or creates and stores it if the buffer is not full
  /*
  boost::shared_ptr<ImuVisual> visual;
  if( visuals_.full() )
  {
    visual = visuals_.front();
  }
  else
  {
    visual.reset(new ImuVisual( context_->getSceneManager(), scene_node_ ));
  }

  // Now set or update the contents of the chosen visual.
  visual->setMessage( msg );
  visual->setFramePosition( position );
  visual->setFrameOrientation( orientation );

  float alpha = alpha_property_->getFloat();
  Ogre::ColourValue color = color_property_->getOgreColor();
  visual->setColor( color.r, color.g, color.b, alpha );

  // And send it to the end of the circular buffer
  visuals_.push_back(visual);*/
//  return;
//}

} // end namespace rviz_plugin_tutorials

// Tell pluginlib about this class.  It is important to do this in
// global scope, outside our package's namespace.
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(cam_display::CamWidgetDisplay,rviz::Display )
// END_TUTORIAL