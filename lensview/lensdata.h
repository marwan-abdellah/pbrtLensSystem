// A simple set of realistic camera lens classes
// Nolan Goodnight <ngoodnight@cs.virginia.edu>
#ifndef _LENSDATA_H_
#define _LENSDATA_H_

#include "lensSphere.h"
#include "lensview.h"

enum mode {thick, precise};

// A simple class for the camera lens interface elements
class lensElement {
public:
    lensElement(void): 
	    isStop(true), rad(0), zpos(0), ndr(0), aper(0), isActive(false) {
		    edge[X] = edge[Y] = 0.0f;
		}
	lensElement(float _rad, float _xpos, float _ndr, float _aper) {
		this->Init(_rad, _xpos, _ndr, _aper);
		edge[X] = edge[Y] = 0.0f;
	}
   
	// Method to initialize a lens interface object
	void Init(float _rad, float _xpos, float _ndr, float _aper) 
	{
		this->isStop = false;
		rad = _rad; zpos = _xpos; 
		ndr = _ndr; aper = _aper;
		this->isActive = true;
		if (rad == 0.0f && ndr == 0.0f)
			this->isStop = true;

		this->lensSph = lensSphere(_rad, -_rad, _rad, 360.0f, _aper, Point(0, 0, _xpos));
	}
	// Draw a single lens interfaces
	void Draw(void);
	bool isAir(void) { return (ndr == 1.0f)?true:false; }

    bool isStop;	// Flag to identify the aperture stop
	lensSphere lensSph;	// Sphere from which lens element was derived
    float rad;		// Radius for the lens interface
	float zpos;		// Lens interface position (1D);
	float ndr;		// Index of refraction
	float aper;		// Aperture (diameter) of the interface
	bool isActive;	// Flag to set interface active
	float edge[2];	// Top edge point of each interface
	int elemNum;	// Stores position in lens system
};

// A simple class for the camera lens systems: lens interfaces
class lensSystem {
public:
	lensSystem(void): f1(0), f2(0), p1(0), p2(0), fstop(1) {
		this->oPlane = INFINITY; this->iPlane = f2; stopElement = 0; stopTrans = 1; this->M = precise;}
	lensSystem(const char *file): f1(0), f2(0), p1(0), p2(0), fstop(1) {
		this->oPlane = INFINITY; this->iPlane = f2; stopElement = 0; stopTrans = 1; this->M = precise;
		this->Load(file); 
	}
	// Method to load a lens specification file and build
	// a vector of lens interfaces. 
	bool Load(const char *file);
	// Get the number of lens interfaces
	int numLenses(void) { return int(lenses.size()); }
	// Draw the entire lens system (all interfaces)
	void Draw(void);
	// Trace a ray through the system
    bool Trace(Ray &ray, Ray* res);
	bool TraceF(Ray &ray, Ray* res, bool draw, int startLens = 0);
	bool TraceB(Ray &ray, Ray* res, bool draw, int endLens = 0);
	// Find intrinsic lens properties
	void findIntrinsics();
	// Trace rays to find (focal point)'
	void findFp();
	// Trace rays to find focal point
	void findF();
	// Trace rays to find (principal axis)
	void findP(const Ray focalRay, const Ray r2, float* p);
	// Refocus the system at a point z
	void refocus(float zPos, int direction);
	// Find the minimum fstop
	void findMinFStop();
	// Find exit pupil
	void findExitPupil();
	// Recalculates aperture
	void recalAper();
	// Calculates thick lens transformation
	void thickLens(Point &e, Point* et, Reference<Matrix4x4> transform);
	// Get the maximum aperture in the lens system
	float maxAperture(void);
	// Get the minimum aperture in the lens system
	float minAperture(void);
	// Get the aperture interface
	lensElement *getAperture(void);

    vector<lensElement> lenses; // Vector of lens elements
	float f1, f2, f1T, f2T;     // Focal points for the lens
	float p1, p2, p1T, p2T;     // Locations of principle planes
	Point pupil;			    // Exit pupil for the lens system
	
	float fstop;		        // Current fstop for the camera
	float maxFS;                // Maximum fstop for the camera
	float minFS;                // Minimum fstop for the camera

	float iPlane;				// Image plane
	float oPlane;				// Object focal plane

	int curLens;				// Dummy to test lens intersection
	int stopElement;
	int stopTrans;				// Controls whether the object can move closer to the lens system
	float thetaMax;
	Reference<Matrix4x4> thick;
	Reference<Matrix4x4> backThick;
	mode M;
};

#endif

