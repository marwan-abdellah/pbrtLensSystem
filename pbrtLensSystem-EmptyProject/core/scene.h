
/*
    pbrt source code Copyright(c) 1998-2010 Matt Pharr and Greg Humphreys.

    This file is part of pbrt.

    pbrt is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.  Note that the text contents of
    the book "Physically Based Rendering" are *not* licensed under the
    GNU GPL.

    pbrt is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

 */

#ifndef PBRT_SCENE_H
#define PBRT_SCENE_H
// scene.h*
#include "pbrt.h"
#include "primitive.h"
#include "transport.h"
// Scene Declarations
class COREDLL Scene {
public:
	// Scene Public Methods
	void Render();
	Scene(Camera *c, SurfaceIntegrator *in,
		VolumeIntegrator *vi, Sampler *s,
		Primitive *accel, const vector<Light *> &lts,
		VolumeRegion *vr);
	~Scene();
	bool Intersect(const Ray &ray, Intersection *isect) const {
		return aggregate->Intersect(ray, isect);
	}
	bool IntersectP(const Ray &ray) const {
		return aggregate->IntersectP(ray);
	}
	const BBox &WorldBound() const;
	Spectrum Li(const RayDifferential &ray, const Sample *sample,
		float *alpha = NULL) const;
	Spectrum Transmittance(const Ray &ray) const;
	// Scene Data
	Primitive *aggregate;
	vector<Light *> lights;
	Camera *camera;
	VolumeRegion *volumeRegion;
	SurfaceIntegrator *surfaceIntegrator;
	VolumeIntegrator *volumeIntegrator;
	Sampler *sampler;
	BBox bound;
};
#endif // PBRT_SCENE_H
