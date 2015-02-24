#include "material.h"
#include "ray.h"
#include "light.h"
#include "../ui/TraceUI.h"
extern TraceUI* traceUI;

#include "../fileio/bitmap.h"
#include "../fileio/pngimage.h"
#include "../vecmath/vec.h"

using namespace std;
extern bool debugMode;

// Apply the phong model to this point on the surface of the object, returning
// the color of that point.
Vec3d Material::shade(Scene *scene, const ray& r, const isect& i) const
{
  // YOUR CODE HERE

  // For now, this method just returns the diffuse color of the object.
  // This gives a single matte color for every distinct surface in the
  // scene, and that's it.  Simple, but enough to get you started.
  // (It's also inconsistent with the phong model...)

  // Your mission is to fill in this method with the rest of the phong
  // shading model, including the contributions of all the light sources.

  // When you're iterating through the lights,
  // you'll want to use code that looks something
  // like this:
  //
  // for ( vector<Light*>::const_iterator litr = scene->beginLights(); 
  // 		litr != scene->endLights(); 
  // 		++litr )
  // {
  // 		Light* pLight = *litr;
  // 		.
  // 		.
  // 		.
  // }
  // You will need to call both the distanceAttenuation() and
  // shadowAttenuation() methods for each light source in order to
  // compute shadows and light falloff.
  
  Vec3d Qpoint = r.at(i.t);
  Vec3d intensity = ke(i) + prod(ka(i), scene->ambient());

  for (vector<Light*>::const_iterator litr = scene->beginLights(); litr != scene->endLights(); ++litr)
  {
    Light* pLight = *litr;
    // Diffuse Term
    Vec3d directionToLight = pLight->getDirection(Qpoint);
    directionToLight.normalize();
    Vec3d lightIntensity = pLight->distanceAttenuation(Qpoint) * pLight->shadowAttenuation(r, Qpoint);
    intensity = intensity + prod(kd(i), lightIntensity) * (i.N * directionToLight);
    // Specular term
    Vec3d viewingDirection = scene->getCamera().getEye() - Qpoint;
    viewingDirection.normalize();
    Vec3d cosVector = i.N * (directionToLight * i.N);
    Vec3d sinVector = cosVector + directionToLight;
    Vec3d reflectedDirection = cosVector + sinVector;
    reflectedDirection.normalize();
    intensity = intensity + prod(ks(i), lightIntensity) * pow((reflectedDirection*viewingDirection), shininess(i));
  }
  return intensity;
}

TextureMap::TextureMap( string filename ) {

	int start = (int) filename.find_last_of('.');
	int end = (int) filename.size() - 1;
	if (start >= 0 && start < end) {
		string ext = filename.substr(start, end);
		if (!ext.compare(".png")) {
			png_cleanup(1);
			if (!png_init(filename.c_str(), width, height)) {
				double gamma = 2.2;
				int channels, rowBytes;
				unsigned char* indata = png_get_image(gamma, channels, rowBytes);
				int bufsize = rowBytes * height;
				data = new unsigned char[bufsize];
				for (int j = 0; j < height; j++)
					for (int i = 0; i < rowBytes; i += channels)
						for (int k = 0; k < channels; k++)
							*(data + k + i + j * rowBytes) = *(indata + k + i + (height - j - 1) * rowBytes);
				png_cleanup(1);
			}
		}
		else
			if (!ext.compare(".bmp")) data = readBMP(filename.c_str(), width, height);
			else data = NULL;
	} else data = NULL;
	if (data == NULL) {
		width = 0;
		height = 0;
		string error("Unable to load texture map '");
		error.append(filename);
		error.append("'.");
		throw TextureMapException(error);
	}
}

Vec3d TextureMap::getMappedValue( const Vec2d& coord ) const
{
  // YOUR CODE HERE

  // In order to add texture mapping support to the 
  // raytracer, you need to implement this function.
  // What this function should do is convert from
  // parametric space which is the unit square
  // [0, 1] x [0, 1] in 2-space to bitmap coordinates,
  // and use these to perform bilinear interpolation
  // of the values.

  return Vec3d(1,1,1);

}


Vec3d TextureMap::getPixelAt( int x, int y ) const
{
    // This keeps it from crashing if it can't load
    // the texture, but the person tries to render anyway.
    if (0 == data)
      return Vec3d(1.0, 1.0, 1.0);

    if( x >= width )
       x = width - 1;
    if( y >= height )
       y = height - 1;

    // Find the position in the big data array...
    int pos = (y * width + x) * 3;
    return Vec3d(double(data[pos]) / 255.0, 
       double(data[pos+1]) / 255.0,
       double(data[pos+2]) / 255.0);
}

Vec3d MaterialParameter::value( const isect& is ) const
{
    if( 0 != _textureMap )
        return _textureMap->getMappedValue( is.uvCoordinates );
    else
        return _value;
}

double MaterialParameter::intensityValue( const isect& is ) const
{
    if( 0 != _textureMap )
    {
        Vec3d value( _textureMap->getMappedValue( is.uvCoordinates ) );
        return (0.299 * value[0]) + (0.587 * value[1]) + (0.114 * value[2]);
    }
    else
        return (0.299 * _value[0]) + (0.587 * _value[1]) + (0.114 * _value[2]);
}

