#include <cmath>

#include "light.h"

using namespace std;

double DirectionalLight::distanceAttenuation(const Vec3d& P) const
{
  // distance to light is infinite, so f(di) goes to 0.  Return 1.
  return 1.0;
}


Vec3d DirectionalLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  return Vec3d(1,1,1);
}

Vec3d DirectionalLight::getColor() const
{
  return color;
}

Vec3d DirectionalLight::getDirection(const Vec3d& P) const
{
  // for directional light, direction doesn't depend on P
  return -orientation;
}

double PointLight::distanceAttenuation(const Vec3d& P) const
{

  // YOUR CODE HERE

  // You'll need to modify this method to attenuate the intensity 
  // of the light based on the distance between the source and the 
  // point P.  For now, we assume no attenuation and just return 1.0

  Vec3d direction = position - P;
  double distanceSq = direction.length2();
  double distance = sqrt(distanceSq);
  return min(1.0, 1/(constantTerm + linearTerm*distance + quadraticTerm*distanceSq));
}

Vec3d PointLight::getColor() const
{
  return color;
}

Vec3d PointLight::getDirection(const Vec3d& P) const
{
  Vec3d ret = position - P;
  ret.normalize();
  return ret;
}


Vec3d PointLight::shadowAttenuation(const ray& r, const Vec3d& p) const
{
  // YOUR CODE HERE:
  // You should implement shadow-handling code here.
  Vec3d shadowDirection = (position-p);
  shadowDirection.normalize();
  isect i;
  ray shadowRay(p, shadowDirection, ray::SHADOW);
  if (this->getScene()->intersect(shadowRay, i))
  {
    double lightDistance = (position-p).length2();
    Vec3d Qpoint = shadowRay.at(i.t);
    double distanceSq = (Qpoint - p).length2();
    if (distanceSq < lightDistance)
    {
      return Vec3d(0,0,0);
    }
  }
  return Vec3d(1,1,1);
}
