#ifndef VECTOR_VISUAL_H
#define VECTOR_VISUAL_H

#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreVector3.h>
#include <decomp_basis/data_type.h>
#include <rviz_rendering/objects/arrow.hpp>

namespace decomp_rviz_plugins {
class VectorVisual {
public:
  VectorVisual(Ogre::SceneManager *scene_manager, Ogre::SceneNode *parent_node);
  ~VectorVisual();

  void setMessage(const vec_E<std::pair<Vec3f, Vec3f>> &vs);
  void setFramePosition(const Ogre::Vector3 &position);
  void setFrameOrientation(const Ogre::Quaternion &orientation);

  void setColor(float r, float g, float b, float a);
  void setScale(float s);

private:
  std::vector<std::unique_ptr<rviz_rendering::Arrow>> objs_;

  Ogre::SceneNode *frame_node_;

  Ogre::SceneManager *scene_manager_;

  float s_ = 1.0;
  vec_E<std::pair<Vec3f, Vec3f>> vs_;
};
} // namespace decomp_rviz_plugins

#endif
