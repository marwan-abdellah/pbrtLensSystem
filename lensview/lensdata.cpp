// A simple set of realistic camera lens classes
// Nolan Goodnight <ngoodnight@cs.virginia.edu>
#include <fstream>
#include "lensdata.h"

#include "../core/geometry.h"
#include <math.h>

#include <stdio.h>
using namespace std;

#include <cmath>

// Method to load a lens specification file and build
// a vector of lens interfaces. 
bool lensSystem::Load(const char *file) {
    ifstream lensfile(file);
    if (!lensfile) return false;

    this->lenses.clear();
    // Iterate over the lens element specs and build
    // the lens system
    float dist = 0.0f, zpos = 0.0f;
    int elem = 0;
    while (!lensfile.eof()) {
        lensElement lens;
        string data;

        lensfile >> data;
        // Remove comments and empty lines
        if (!data.compare("#") || !data.compare("")) {
            getline(lensfile, data);
            continue;
        }
        float rad  = (float)atof(data.c_str());
        lensfile >> data;
        float axpos = dist - zpos;
        dist -= zpos;
        zpos = (float)atof(data.c_str());

        lensfile >> data;
        float ndr  = (float)atof(data.c_str());
        lensfile >> data;
        float aper = (float)atof(data.c_str());
        if (!aper && !rad && !zpos && !ndr)
            return false;

        // Initialize the lens interface and store
        lens.Init(rad, axpos, ndr, aper);
        lens.elemNum = elem++;
        this->lenses.push_back(lens);
    }
    if (this->numLenses() < 5)
        return false;
    this->curLens = this->numLenses() - 1;
    return true;
}

// Method to intersect two rays
void intersectRay(Ray r1, Ray r2, float* tHit)
{
    *tHit = -(r2.d.y*r1.o.z - r2.d.z*r1.o.y - r2.d.y*r2.o.z + r2.d.z*r2.o.y)/(r1.d.z*r2.d.y - r1.d.y*r2.d.z);
}

// Find exit pupil
void lensSystem::findExitPupil()
{
    lensElement* aperture = this->getAperture();
    Point e = Point(0, aperture->aper/2, aperture->zpos);
    Point i;
    e.z -= this->p1T;
    this->thickLens(e, &i, this->backThick);
    i.z += this->p1T;

    //drawPoint(5, e.z+this->p1T, e.y);
    //glColor3f(1.0, 0.0, 0.0);
    //drawPoint(5, i.z, i.y);

    this->pupil.z = i.z;
    this->pupil.y = i.y*2.0;
}

// Calculates thick lens transformation
void lensSystem::thickLens(Point &e, Point* et, Reference<Matrix4x4> transform)
{
    Point ef = e;

    float ePoint[4] = {ef.x, ef.y, ef.z, 1.0};
    float m[4] = {0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            m[i] += transform->m[i][j] * ePoint[j];

    *et = Point(m[0]/m[3], m[1]/m[3], (m[2])/m[3]);
}

// Refocus the system at a point z
void lensSystem::refocus(float zPos, int direction)
{
    //float T0;
    //float T1;
    //float fp = (this->f2 - this->p2);//(-68.1 + 17.8);//
    //float t = (this->p2 - this->p1);//abs(-23.2 + 17.8);//
    //float z = zPos + this->f2;	//zPos + this->iPlane;//

    //if (Quadratic(1, (2*fp + t - z), fp*fp, &T0, &T1))
    //{
    //	this->iPlane = this->f2 - min(T0, T1);//-68.1 - min(T0, T1);//
    //	printf("z: %4.2f\n", z);
    //}
    float avgZ = 0.0;
    int counter = 0;
    printf("refocus - 1\n");
    if ((this->iPlane <= this->f2 - 70.0 || zPos <= 10) && direction == -1)
    {
        this->stopTrans = 0;
    } else if (direction == 1)
    {
        this->stopTrans = 1;
    }

    printf("refocus - 2\n");

    for (int i = -8; i <= 8; i++)
    {
        printf("refocus - 3 - 1\n");
        Ray focalRay = Ray(Point(0, 0, zPos), Vector(0, this->getAperture()->aper/(i*2), -zPos));
        printf("refocus - 3 - 2\n");
        Ray lensRay;
        printf("refocus - 3 - 3\n");
        TraceF(focalRay, &lensRay, false);
        float tHit;
        printf("refocus - 3 - 4\n");
        intersectRay(lensRay, Ray(Point(0,0,0), Vector(0,0,-1)), &tHit);
        printf("refocus - 3 - 5\n");
        if (tHit > 0)
        {
            //printf("%4.2f\n", tHit);
            //this->iPlane = lensRay(tHit).z;
            avgZ += lensRay(tHit).z;
            printf("refocus - 3 - 6\n");
            counter++;
            //if (lensRay(tHit).z <= this->f2-99)
            //this->stopTrans = 0;
        }
    }
    printf("refocus - 3 - 7\n");
    avgZ /= counter;
    this->iPlane = avgZ;
}

// Find intrinsic lens properties
void lensSystem::findIntrinsics()
{
    printf("findIntrinsics - 1\n");
    this->findFp();
    printf("findIntrinsics - 2\n");
    this->findF();
    printf("findIntrinsics - 3\n");
    this->findMinFStop();
    printf("findIntrinsics - 4\n");
    float t = this->p2T - this->p1T;
    float a = 1.0 + t / (this->f2T - this->p2T);
    float b = 1.0 / (this->f2T - this->p2T);

    backThick = new Matrix4x4(	1.0,	0.0,	0.0,	0.0,
                                0.0,	1.0,	0.0,	0.0,
                                0.0,	0.0,	a,		t,
                                0.0,	0.0,	b,		1.0);

    t = this->p2 - this->p1;
    a = 1.0 + t / (this->f1 - this->p1);
    b = 1.0 / (this->f1 - this->p1);

    thick = new Matrix4x4(	1.0,	0.0,	0.0,	0.0,
                            0.0,	1.0,	0.0,	0.0,
                            0.0,	0.0,	a,		t,
                            0.0,	0.0,	b,		1.0);

    printf("findIntrinsics - 5\n");
    this->recalAper();
    printf("findIntrinsics - 6\n");
    this->findExitPupil();
    printf("findIntrinsics - 7\n");
}

// Recalculates aperture
void lensSystem::recalAper()
{
    float theta = asin(1.0f/(2.0f*this->fstop));
    Ray r1 = Ray(Point(0.0f, 0.0f, this->f2), Vector(0.0f, (this->lenses[this->lenses.size()-1].zpos - this->f2)*tan(theta), this->lenses[this->lenses.size()-1].zpos - this->f2));
    drawLine(r1(0).z, r1(0).y, r1(10).z, r1(10).y);
    Ray r2;
    lensElement* aperture = this->getAperture();
    aperture->aper += 10;
    float tHit;

    // Trace ray at max theta through system
    TraceB(r1, &r2, true, aperture->elemNum);
    // Find intersection with aperture interface
    intersectRay(r2, Ray(Point(0, 0, aperture->zpos), Vector(0, 1, 0)), &tHit);
    // Update aperture size
    aperture->aper = r2(tHit).y*2;
    //glColor3f(1.0, 0.0, 0.0);
    //drawLine(r2(0).z, r2(0).y, r2(10).z, r2(10).y);
}




// Find the minimum fstop
void lensSystem::findMinFStop()
{
    printf("findMinFStop - 1\n");
    lensElement* stop = this->getAperture();
    printf("findMinFStop - 2\n");
    float xHit = 0.0;
    Ray aperRay;
    for (int i = 0; i < 1000; i++)
    {
        float theta = (float) i / 2000.0 * M_PI;
        float yDir = tan(theta);
        Ray topAp = Ray(Point(0, stop->aper/2 - .05, stop->zpos), Vector(0,-yDir,-1));
        Ray botAp = Ray(Point(0, -stop->aper/2 + .05, stop->zpos), Vector(0,yDir,-1));
        float rayIntersection;
        printf ("rayIntersection %f \n", rayIntersection);

        Ray topRes;
        Ray botRes;
        glColor3f(1.0, 0.5, 0.25);
        printf("findMinFStop - 3\n");
        TraceF(topAp, &topRes, false, this->stopElement+1);
        printf("findMinFStop - 4\n");
        TraceF(botAp, &botRes, false, this->stopElement+1);
        printf("findMinFStop - 5\n");
        intersectRay(topRes, botRes, &rayIntersection);
        printf("findMinFStop - 6\n");

        printf("rayIntersection %f \n", rayIntersection);
        if (rayIntersection != rayIntersection )
        {
            printf("chamnge the value \n");
            rayIntersection = 0.0000001f;

        }


        printf("rayIntersection %9.9f \n", rayIntersection);
        if (topRes(rayIntersection).y == 0)
        {
            printf("findMinFStop - 7 - 1 \n");
            float val1 = (topRes(rayIntersection).z - this->f2);

            printf("findMinFStop - 7 - 2\n");
            float val2 = (xHit - this->f2);
            //drawLine(topRes(0).x, topRes(0).y, topRes(50).x, topRes(50).y);
            printf("findMinFStop - 7 - 3\n");
            if (abs(val1) < abs(val2))
            {
                printf("findMinFStop - 7 - 4\n");
                xHit = topRes(rayIntersection).z;
                aperRay = topRes;
            }
        }
        printf("findMinFStop - 8 -1 \n");
    }
    printf("findMinFStop - 8\n");
    Vector aperVec = Vector(0, aperRay(0).y, aperRay(0).z-xHit);
    float theta = acos(Dot(aperVec, Vector(0,0,1)) / aperVec.Length());
    this->thetaMax = theta;
    this->minFS = 1.0 / ( 2.0 * sin(theta));
    this->fstop = this->minFS;
}

// Trace a ray through the system
bool lensSystem::Trace(Ray &ray, Ray* res)
{
    if (this->M == precise)
    {
        if (ray(0).z - ray(1).z < 0)
            return TraceB(ray, res, true);
        else
            return TraceF(ray, res, true);
    } else
    {
        Ray r1 = ray;
        float tHit;
        float pTest = min(this->p2, this->p1);
        intersectRay(r1, Ray(Point(0,-this->maxAperture(),pTest), Vector(0,1,0)), &tHit);
        if (tHit < 0)
            return false;
        drawLine(r1(0).z, r1(0).y, r1(tHit).z, r1(tHit).y);
        drawLine(r1(tHit).z, r1(tHit).y, max(this->p2, this->p1), r1(tHit).y);
        r1.o.z -= this->p2;
        thickLens(r1.o, &(*res).o, this->thick);
        (*res).o.z += this->p2;
        drawLine(max(this->p2, this->p1), r1(tHit).y, (*res).o.z, (*res).o.y);
        drawPoint(5, (*res).o.z, (*res).o.y);
    }
}

// Trace rays to find (focal point)'
void lensSystem::findFp()
{
    Ray focalRay;
    //Only really need to do this once for the system
    for (int i = 0; i < 100000; i++)
    {
        Ray testRay = Ray(Point(0.0, this->minAperture() / 8.0 - (float) i/1000.0, 1.0), Vector(0.0, 0.0, -1.0));
        //glColor3f(0.0, 1.0, 0.0);
        if (TraceF(testRay, &focalRay, false))
        {
            // Compute focal point
            float fp = focalRay.o.z + (-focalRay.o.y/focalRay.d.y)*focalRay.d.z;
            // Compute principal axis'
            this->findP(focalRay, testRay, &(this->p2));
            //printf("F': %4.2f\n", fp);
            this->f2 = fp;
            this->iPlane = fp; // -68.1
            break;
        }
    }
    focalRay = Ray();
    //Find f' and P' for back half
    for (int i = 0; i < 100000; i++)
    {
        Ray testRay = Ray(Point(0.0, this->minAperture() / 8.0 - (float) i/1000.0, 1.0), Vector(0.0, 0.0, -1.0));
        glColor3f(1.0, 0.0, 0.0);
        if (TraceF(testRay, &focalRay, false, getAperture()->elemNum))
        {
            // Compute focal point
            float fpT = focalRay.o.z + (-focalRay.o.y/focalRay.d.y)*focalRay.d.z;
            drawPoint(5, fpT, 0);
            // Compute principal axis'
            this->findP(focalRay, testRay, &(this->p2T));
            //printf("F': %4.2f\n", fp);
            this->f2T = fpT;
            break;
        }
    }
}

// Trace rays to find focal point
void lensSystem::findF()
{
    Ray focalRay;
    //Only really need to do this once for the system
    for (int i = 0; i < 100000; i++)
    {
        Ray testRay = Ray(Point(0.0, this->minAperture() / 8.0 - (float) i/1000.0, this->lenses[this->lenses.size()-1].zpos - 1.0), Vector(0.0, 0.0, 1.0));
        //glColor3f(0.0, 1.0, 0.0);
        if (TraceB(testRay, &focalRay, false))
        {
            // Compute focal point
            float f = focalRay.o.z + (-focalRay.o.y/focalRay.d.y)*focalRay.d.z;
            // Compute principal axis
            this->findP(focalRay, testRay, &(this->p1));
            //printf("F: %4.2f\n", f);
            this->f1 = f;
            break;
        }
    }
    focalRay = Ray();
    // Find f and P for back half of lens system
    for (int i = 0; i < 100000; i++)
    {
        Ray testRay = Ray(Point(0.0, this->minAperture() / 8.0 - (float) i/1000.0, this->lenses[this->lenses.size()-1].zpos - 1.0), Vector(0.0, 0.0, 1.0));
        glColor3f(1.0, 0.0, 0.0);
        if (TraceB(testRay, &focalRay, false, getAperture()->elemNum))
        {
            // Compute focal point
            float fT = focalRay.o.z + (-focalRay.o.y/focalRay.d.y)*focalRay.d.z;
            drawLine(focalRay(0).z, focalRay(0).y, focalRay(100).z, focalRay(100).y);
            drawPoint(5, fT, 0);
            // Compute principal axis
            this->findP(focalRay, testRay, &(this->p1T));
            //printf("F: %4.2f\n", f);
            this->f1T = fT;
            break;
        }
    }
}

// Trace rays to find principal axis
void lensSystem::findP(const Ray focalRay, const Ray r2, float* p)
{
    Ray r1 = Ray(focalRay(-focalRay.o.y/focalRay.d.y), -focalRay.d);
    float t1;
    intersectRay(r1, r2, &t1);
    //drawLine(r1(0).x, r1(0).y, r1(100).x, r1(100).y);
    //drawLine(r2(0).x, r2(0).y, r2(100).x, r2(100).y);

    *p = r1(t1).z;
}

// Trace a ray forward (from focal plane) through the system
bool lensSystem::TraceF(Ray &ray, Ray* res, bool draw, int startLens)
{
    printf("TraceF - 1\n");
    float tHit;
    Ray r = ray;
    //drawLine(ray(0).x, ray(0).y, ray(1).x, ray(1).y);
    Vector n;
    this->curLens = max(this->curLens, 0);
    printf("TraceF - 2\n");
    int lensSize = this->lenses.size();
    this->curLens = min(this->curLens, lensSize);
    printf("TraceF - 3\n");
    //int i = this->curLens;
    int lensSizess = this->lenses.size();

    if (lensSizess > 0)
    {
        for (int i = startLens; i < lensSizess ; i++)
        {
            printf("TraceF - 4\n");
            if(this->lenses[i].isStop)
            {
                printf("TraceF - 5\n");
                float phiMax = Radians(360.0f);
                printf("TraceF - 6\n");
                //r.o = r.o - Vector(center) + negativeRad * Vector(Point(radius,0,0));
                // Compute plane intersection for disk
                printf("TraceF - 7\n");
                if (fabsf(r.d.z) < 1e-7) return false;
                printf("TraceF - 8\n");
                float thit = (this->lenses[i].zpos - r.o.z) / r.d.z;
                if (thit < r.mint || thit > r.maxt)
                    return false;

                printf("TraceF - 9\n");
                // See if hit point is inside disk radii and \phimax
                Point phit = r(thit);
                printf("TraceF - 10\n");
                float dist2 = phit.x * phit.x + phit.y * phit.y;
                if (dist2 > (this->lenses[i].aper/2) * (this->lenses[i].aper/2) ||
                        dist2 < 0)
                    return false;
                printf("TraceF - 11\n");
                // Test disk $\phi$ value against \phimax
                float phi = atan2f(phit.y, phit.x);
                printf("TraceF - 12\n");
                if (phi < 0) phi += 2. * M_PI;

                if (phi > phiMax)
                    return false;
            }
            else if(this->lenses[i].lensSph.Intersect(r, &tHit, &n) && !this->lenses[i].isStop)
            {
                if (draw)
                    drawLine(r(0).z, r(0).y, r(tHit).z, r(tHit).y);

                float prevN = i == 0 || this->lenses[i-1].ndr == 0 ? 1 : this->lenses[i-1].ndr;
                float mu = prevN / this->lenses[i].ndr;

                Vector rayHit = Vector(r(tHit)-r(0));

                float gamma = mu - 1;
                r.o = r(tHit);
                r.d = mu * Normalize(rayHit) - gamma * Normalize(n);
                if ((i == this->lenses.size()-1) && draw)
                    drawLine(r(0).z, r(0).y, r(100).z, r(100).y);
                //printf("HIT %0.2f: (%0.2f, %0.2f)\n", ray.o.y, ray(tHit).x, ray(tHit).y);
            } else
            {
                return false;
                //drawLine(ray(0).x, ray(0).y, ray(INFINITY).x, ray(INFINITY).y);
            }
        }
    }



    *res = r;
    return true;
}

// Trace a ray backward (from image plane) through the system
bool lensSystem::TraceB(Ray &ray, Ray* res, bool draw, int endLens)
{
    float tHit;
    Ray r = ray;
    Vector n;
    this->curLens = std::max(this->curLens, 0);

    int sizeLens = this->lenses.size();
    this->curLens = std::min(this->curLens, sizeLens);
    //int i = this->curLens;
    for (int i = this->lenses.size() - 1; i >= endLens; i--)
    {
        if(this->lenses[i].isStop)
        {
            float phiMax = Radians(360.0f);
            lensElement aperture = this->lenses[i];
            // Compute plane intersection for disk
            if (fabsf(r.d.z) < 1e-7) return false;
            float thit = (aperture.zpos - r.o.z) / r.d.z;
            if (thit < r.mint || thit > r.maxt)
                return false;
            // See if hit point is inside disk radii and \phimax
            Point phit = r(thit);
            float dist2 = phit.x * phit.x + phit.y * phit.y;
            if (dist2 > (aperture.aper/2) * (aperture.aper/2) ||
                    dist2 < 0)
                return false;
            // Test disk $\phi$ value against \phimax
            float phi = atan2f(phit.y, phit.x);
            if (phi < 0) phi += 2. * M_PI;
            if (phi > phiMax)
                return false;
        }
        else if(this->lenses[i].lensSph.Intersect(r, &tHit, &n))
        {
            if (draw)
                drawLine(r(0).z, r(0).y, r(tHit).z, r(tHit).y);
            float nextN = i == 0 || this->lenses[i-1].ndr == 0 ? 1 : this->lenses[i-1].ndr;
            float mu = this->lenses[i].ndr / nextN;

            Vector rayHit = Vector(r(tHit)-r(0));

            float gamma = mu - 1;
            r.o = r(tHit);
            r.d = mu * Normalize(rayHit) + gamma * Normalize(n);
            if (i == 0 && draw)
                drawLine(r(0).z, r(0).y, r(300).z, r(300).y);
        } else
        {
            return false;
        }
    }
    *res = r;
    return true;
}

// Draw the entire lens system (all interfaces and aperture)
void lensSystem::Draw(void) {

    glColor3f(0.75f, 0.75f, 0.75f); glLineWidth(1);
    vector<lensElement>::iterator Li, Ln;

    for (Li = lenses.begin(); Li != lenses.end(); Li++)
        if (Li->isActive && !Li->isStop)
            Li->Draw();

    for (Li = lenses.begin(); Li != lenses.end() - 1; Li++) {
        Ln = Li + 1;
        if ((Ln->isStop || Li->isStop) || Li->isAir() || !Li->isActive)
            continue;

        float lip[2] = {Li->edge[X], Li->edge[Y]};
        float lnp[2] = {Ln->edge[X], Ln->edge[Y]};

        if (Li->aper < Ln->aper) {
            drawLine(lip[X],  lip[Y], lip[X],  lnp[Y]);
            drawLine(lnp[X],  lnp[Y], lip[X],  lnp[Y]);
            drawLine(lip[X], -lip[Y], lip[X], -lnp[Y]);
            drawLine(lnp[X], -lnp[Y], lip[X], -lnp[Y]);
        } else {
            drawLine(lip[X],  lip[Y], lnp[X],  lip[Y]);
            drawLine(lnp[X],  lnp[Y], lnp[X],  lip[Y]);
            drawLine(lip[X], -lip[Y], lnp[X], -lip[Y]);
            drawLine(lnp[X], -lnp[Y], lnp[X], -lip[Y]);
        }
    }

    // Draw the aperture stop interface
    lensElement *stop = this->getAperture();
    float maxp = this->maxAperture();
    float haxp = stop->aper / 2.0f;
    float taxp = haxp + maxp / 2.0f;
    glColor3f(1, 1, 1);
    glLineWidth(2);
    drawLine(stop->zpos, haxp, stop->zpos, taxp);
    drawLine(stop->zpos, -haxp, stop->zpos, -taxp);
    glLineWidth(1);
}

// Get the maximum aperture in the lens system
float lensSystem::maxAperture(void) {
    float dist = 0.0f;
    vector<lensElement>::iterator Li;
    for (Li = lenses.begin(); Li != lenses.end(); Li++)
        if (Li->aper > dist)
            dist = Li->aper;
    return dist;
}

// Get the maximum aperture in the lens system
float lensSystem::minAperture(void) {
    float dist = INFINITY;
    vector<lensElement>::iterator Li;
    for (Li = lenses.begin(); Li != lenses.end(); Li++)
        if (Li->aper < dist)
            dist = Li->aper;
    return dist;
}

// Method to get the aperture interface
lensElement* lensSystem::getAperture(void) {
    int i;
    for (i = 0; i < numLenses(); i++)
        if (lenses[i].isStop)
        {
            this->stopElement = i;
            return (&lenses[i]);
        }
    return NULL;
}

// Draw a single lens interface
void lensElement::Draw(void) 
{
    float nextX, nextY, thisX, thisY;
    thisY = -this->aper / 2.0f;
    float rs = this->rad / fabs(this->rad);
    int i;

    float step = this->aper / LVSD;
    float posx = sqrt(rad*rad - thisY*thisY);
    thisX = rs * posx - rad + zpos;

    for (i = 1; i <= LVSD; i++) {
        nextY = thisY + step;
        posx = sqrt(rad*rad - nextY*nextY);
        nextX = rs * posx - rad + zpos;
        drawLine(thisX, thisY, nextX, nextY);
        thisX = nextX; thisY = nextY;
    }
    this->edge[X] = thisX;
    this->edge[Y] = thisY;
}

