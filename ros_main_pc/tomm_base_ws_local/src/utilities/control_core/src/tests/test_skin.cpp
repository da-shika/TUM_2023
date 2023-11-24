#include <control_core/types.h>
#include <geometry_msgs/PolygonStamped.h>

int main()
{
  // footprint
  cc::Scalar lxn, lxp, lyn, lyp;
  lxn = -0.1;
  lxp = 0.12;
  lyn = -0.08;
  lyp = 0.08;

  //////////////////////////////////////////////////////////////////////////////

  cc::PolygonShape polygon(lxn, lxp, lyn, lyp);

  std::cout << "local vertices=\n" << polygon.vertices() << std::endl;
  std::cout << "maxVertex=\n" << polygon.maxVertex() << std::endl;
  std::cout << "minVertex=\n" << polygon.minVertex() << std::endl;
  std::cout << "centroid=\n" << polygon.centroid() << std::endl;
  std::cout << "halfExtends=\n" << polygon.halfExtends() << std::endl;
  std::cout << "halfSpace A=\n" << polygon.halfSpace().first << std::endl;
  std::cout << "halfSpace b=\n" << polygon.halfSpace().second << std::endl;

  //////////////////////////////////////////////////////////////////////////////

  cc::SkinModality modality;
  modality.area() = 1.0;
  modality.hull() = polygon;
  modality.centerOfPressure() << polygon.centroid(), 0.0;
  modality.wrench().setZero();

  control_core_msgs::SkinModality modality_msg = modality.toSkinModalityMsg();
  for(const auto p : modality_msg.hull.points)
    std::cout << "modality_msg.hull: " << p.x << " " << p.y << " " << p.z << std::endl;
  std::cout << std::endl;

  //////////////////////////////////////////////////////////////////////////////

  cc::SkinPatch patch;
  patch.force() = modality;
  patch.proximity() = modality;

  control_core_msgs::SkinPatch patch_msg = patch.toSkinPatchMsg();


  for(const auto p : patch_msg.force.hull.points)
    std::cout << "patch_msg.hull: " << p.x << " " << p.y << " " << p.z << std::endl;
  std::cout << std::endl;

  cc::SkinPatch patch_2;
  cc::Points2d vertices = cc::Points2d::Zero(2, 3);
  patch_2.force().hull() = cc::PolygonShape(vertices);
  patch_2.proximity().hull() = cc::PolygonShape(vertices);

  std::cout << "patch_2.hull=\n" << patch_2.force().hull().vertices() << std::endl;
  patch_2 = patch;
  std::cout << "patch_2.hull=\n" << patch_2.force().hull().vertices() << std::endl;

  //////////////////////////////////////////////////////////////////////////////
  geometry_msgs::PolygonStamped polygon_msg;
  polygon_msg.polygon = patch.force().hull();

  for(const auto p : polygon_msg.polygon.points)
    std::cout << "polygon_msg.hull: " << p.x << " " << p.y << " " << p.z << std::endl;
  std::cout << std::endl;

  return 0;
}
