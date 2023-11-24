#include <control_core/types/body_id.h>

namespace control_core
{

  const std::vector<std::string>& BodyId::Names()
  {
    const static std::vector<std::string> names = {
      "left_foot", "right_foot", "left_hand", "right_hand", "left_elbow", "right_elbow", "torso", "head"};
    return names;
  }

  const std::vector<std::string>& BodyId::EndEffectorsNames()
  {
    const static std::vector<std::string> names = {
      "left_foot", "right_foot", "left_hand", "right_hand"};
    return names;
  }

  const std::vector<std::string>& BodyId::FootNames()
  {
    const static std::vector<std::string> names = {
      "left_foot", "right_foot"};
    return names;
  }

  const std::vector<std::string>& BodyId::HandNames()
  {
    const static std::vector<std::string> names = {
      "left_hand", "right_hand"};
    return names;
  }

  const std::vector<std::string>& BodyId::ElbowNames()
  {
    const static std::vector<std::string> names = {
      "left_elbow", "right_elbow"};
    return names;
  }

  const std::vector<std::string>& BodyId::UpperBodyNames()
  {
    const static std::vector<std::string> names = {
      "left_hand", "right_hand", "left_elbow", "right_elbow", "torso", "head"};
    return names;
  }

  const std::vector<BodyId::Value>& BodyId::Ids()
  {
    const static std::vector<BodyId::Value> ids = {
      BodyId::LEFT_FOOT, BodyId::RIGHT_FOOT, 
      BodyId::LEFT_HAND, BodyId::RIGHT_HAND, 
      BodyId::LEFT_ELBOW, BodyId::RIGHT_ELBOW, 
      BodyId::HEAD, BodyId::TORSO};
    return ids;
  }

  const std::vector<BodyId::Value>& BodyId::EndEffectorsIds()
  {
    const static std::vector<BodyId::Value> ids = {
      BodyId::LEFT_FOOT, BodyId::RIGHT_FOOT, 
      BodyId::LEFT_HAND, BodyId::RIGHT_HAND};
    return ids;
  }

  const std::vector<BodyId::Value>& BodyId::FootIds()
  {
    const static std::vector<BodyId::Value> ids = {
      BodyId::LEFT_FOOT, BodyId::RIGHT_FOOT};
    return ids;
  }

  const std::vector<BodyId::Value>& BodyId::HandIds()
  {
    const static std::vector<BodyId::Value> ids = {
      BodyId::LEFT_HAND, BodyId::RIGHT_HAND};
    return ids;
  }

  const std::vector<BodyId::Value>& BodyId::ElbowIds()
  {
    const static std::vector<BodyId::Value> ids = {
      BodyId::LEFT_ELBOW, BodyId::RIGHT_ELBOW};
    return ids;
  }

  const std::vector<BodyId::Value>& BodyId::UpperBodyIds()
  {
    const static std::vector<BodyId::Value> ids = {
      BodyId::LEFT_HAND, BodyId::RIGHT_HAND, 
      BodyId::LEFT_ELBOW, BodyId::RIGHT_ELBOW, 
      BodyId::HEAD, BodyId::TORSO};
    return ids;
  }

}