/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreVector3.h>
#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>

#include <rviz/ogre_helpers/line.h>

#include "rviz_cam_display/CameraWidget_visual.h"

namespace cam_display
{

SelfIllumLine::SelfIllumLine( 
    Ogre::SceneManager * manager,
    Ogre::SceneNode * parent_node
  ): rviz::Line(manager, parent_node)
{
  //manual_object_material_->getTechnique(0)->setLightingEnabled(true);
  return;
}


void SelfIllumLine::setColor(const Ogre::ColourValue& c ) {
  manual_object_material_->getTechnique(0)->setSelfIllumination( c );
}

// BEGIN_TUTORIAL
CamWidgetVisual::CamWidgetVisual( Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node )
{
  scene_manager_ = scene_manager;

  // Ogre::SceneNode s form a tree, with each node storing the
  // transform (position and orientation) of itself relative to its
  // parent.  Ogre does the math of combining those transforms when it
  // is time to render.
  //
  // Here we create a node to store the pose of the Imu's header frame
  // relative to the RViz fixed frame.

  frame_node_ = parent_node->createChildSceneNode();
  pyramid_node_ = frame_node_->createChildSceneNode();

  // We create the arrow object within the frame node so that we can
  // set its position and direction relative to its header frame.




  tll_ = new SelfIllumLine(scene_manager, pyramid_node_);
  trl_ = new SelfIllumLine(scene_manager, pyramid_node_);
  brl_ = new SelfIllumLine(scene_manager, pyramid_node_);
  bll_ = new SelfIllumLine(scene_manager, pyramid_node_);
  t_l_ = new SelfIllumLine(scene_manager, frame_node_);
  l_l_ = new SelfIllumLine(scene_manager, frame_node_);
  b_l_ = new SelfIllumLine(scene_manager, frame_node_);
  r_l_ = new SelfIllumLine(scene_manager, frame_node_);




  //std::cout << frame_node_->numChildren() << std::endl;

}

CamWidgetVisual::~CamWidgetVisual()
{
  // Destroy the frame node since we don't need it anymore.
  
  delete tll_;
  delete trl_;
  delete brl_;
  delete bll_;

  delete t_l_;
  delete r_l_;
  delete b_l_;
  delete l_l_;
  scene_manager_->destroySceneNode( pyramid_node_ );
  scene_manager_->destroySceneNode( frame_node_ );
  //std::cout << "got detroyed" << std::endl; 
}

// Position and orientation are passed through to the SceneNode.
void CamWidgetVisual::setFramePosition( const Ogre::Vector3& position )
{
  frame_node_->setPosition( position );
  return;
}

void CamWidgetVisual::setFrameOrientation( const Ogre::Quaternion& orientation )
{
  frame_node_->setOrientation( orientation );
  return;
}

void CamWidgetVisual::setPyramidVisibility(bool pyramidVisibility) {
  pyramid_node_->setVisible(pyramidVisibility);
  return;
}

void CamWidgetVisual::setProjection(double fx, double fy, double px, double py, double s, int w, int h) {
  float y0 = -py/fy;
  float y1 = (h-py)/fy;
  float x0 = -px/fx;
  float x1 = (w-px)/fx;
  float sfx = s/fx;
  float s0 = y0*sfx;
  float s1 = y1*sfx;

  Ogre::Vector3 tl( x1-s1, y1, 1.0 );
  Ogre::Vector3 tr( x0-s1, y1, 1.0 );
  Ogre::Vector3 br( x0-s0, y0, 1.0 );
  Ogre::Vector3 bl( x1-s0, y0, 1.0 );
  Ogre::Vector3  o(  0.0,  0.0, 0.0 );

  tll_->setPoints(o, tl);
  trl_->setPoints(o, tr);
  brl_->setPoints(o, br);
  bll_->setPoints(o, bl);

  t_l_->setPoints(tl, tr);
  r_l_->setPoints(tr, br);
  b_l_->setPoints(br, bl);
  l_l_->setPoints(bl, tl);

  return;
}

void CamWidgetVisual::setScale(float scale) {
  frame_node_->setScale( scale, scale, scale );
  return;
}

// Color is passed through to the Arrow object.
void CamWidgetVisual::setColor( float r, float g, float b, float a )
{

  tll_->setColor(r,g,b,a);
  trl_->setColor(r,g,b,a);
  brl_->setColor(r,g,b,a);
  bll_->setColor(r,g,b,a);

  t_l_->setColor(r,g,b,a);
  r_l_->setColor(r,g,b,a);
  b_l_->setColor(r,g,b,a);
  l_l_->setColor(r,g,b,a);


}
// END_TUTORIAL

} // end namespace rviz_plugin_tutorials

