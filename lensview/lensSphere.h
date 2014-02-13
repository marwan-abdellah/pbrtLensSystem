/*
 * pbrt source code Copyright(c) 1998-2005 Matt Pharr and Greg Humphreys
 *
 * All Rights Reserved.
 * For educational use only; commercial use expressly forbidden.
 * NO WARRANTY, express or implied, for this software.
 * (See file License.txt for complete license)
 */

#include <vector>
#include <geometry.h>

class lensSphere
{
public:
	// Sphere Public Methods
	lensSphere()
	{
        lensSphere(1.0, -1.0, 1.0, `2*M_PI, 1.0, Point(0,0,0));
	}

	lensSphere(float rad, float z0, float z1, float pm, float aper, Point c)
	{
		negativeRad = rad < 0 ? -1 : 1;
		radius = abs(rad);
		zmin = Clamp(min(z0, z1), -radius, radius);
		zmax = Clamp(max(z0, z1), -radius, radius);
		thetaMin = acosf(Clamp(zmin/radius, -1.f, 1.f));
		thetaMax = acosf(Clamp(zmax/radius, -1.f, 1.f));
		phiMax = Radians(Clamp(pm, 0.0f, 360.0f));
		aperture = aper;
		center = c;
	}

	bool Intersect(const Ray &r, float *tHit, Vector* n)
	{
		float phi;
		Point phit;	
		// Transform _Ray_ to object space
		Ray ray = r;
		// Warning: This assumes that the lens is centered on the x-y axis
		ray.o = ray.o - Vector(center) + negativeRad * Vector(Point(0,0,radius));
		int dir = (ray(1).z - ray(0).z) < 0 ? -1 : 1;
		// Compute quadratic sphere coefficients
		float A = ray.d.x*ray.d.x + ray.d.y*ray.d.y +
				ray.d.z*ray.d.z;
		float B = 2 * (ray.d.x*ray.o.x + ray.d.y*ray.o.y +
					ray.d.z*ray.o.z);
		float C = ray.o.x*ray.o.x + ray.o.y*ray.o.y +
				ray.o.z*ray.o.z - radius*radius;
		// Solve quadratic equation for _t_ values
		float t0, t1;
		if (!Quadratic(A, B, C, &t0, &t1))
			return false;
		// Compute intersection distance along ray
		if (t0 > ray.maxt || t1 < ray.mint)
			return false;
		// Negative radius implies concave sphere
		float thit = -dir * negativeRad < 0 ? t1 : t0;
		if (t0 < ray.mint) {
			thit = t1;
			if (thit > ray.maxt) return false;
		}

		// The display is misleading, the actual longitudinal component is x, not z
		float rayrad = ray(thit).x * ray(thit).x + ray(thit).y * ray(thit).y;
		float aperad = (aperture/2) * (aperture/2);

		// Test ray intersection against aperture
		if ( rayrad > aperad)
			return false;

		// Compute sphere hit position and $\phi$
		phit = ray(thit);
		phi = atan2f(phit.y, phit.z);
		if (phi < 0.) phi += 2.f*M_PI;
		// Test sphere intersection against clipping parameters
		if (phit.x < zmin || phit.x > zmax || phi > phiMax) 
		{
			if (thit == t1) return false;
			if (t1 > ray.maxt) return false;
			thit = t1;
			// Compute sphere hit position and $\phi$
			phit = ray(thit);
			phi = atan2f(phit.y, phit.z);
			if (phi < 0.) phi += 2.f*M_PI;
			if (phit.x < zmin || phit.x > zmax || phi > phiMax)
				return false;
		}
		// Update _tHit_ for quadric intersection
		*tHit = thit;
		*n = -negativeRad * Vector(Point(ray(thit).x, ray(thit).y, ray(thit).z));
		return true;
	}
	int negativeRad;
private:
	// Sphere Private Data
	float radius;
	float phiMax;
	float zmin, zmax;
	float thetaMin, thetaMax;	
	float aperture;
	Point center;
};
