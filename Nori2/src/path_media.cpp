#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracingMedia : public Integrator {
public:
	PathTracingMedia(const PropertyList& props) {
		/* No parameters this time */
	}

    /**
     * \brief This implementation will deal with the whole process of ray marching inside the medium, from the point of
     * intersection with the medium until it exits or until it gets extinguished by russian roulette
    */
    Color3f rayMarching(const Scene* scene, Sampler* sampler, const Ray3f& ray, int depth=0) const {
        //printf("rayMarching\n");
        Color3f Lo(0.0f);   // the radiance we will return
        Intersection itsCheck;
        bool intersects = scene->rayIntersect(ray, itsCheck);
        if (!intersects || itsCheck.medium == nullptr) {
            // if the ray doesnt itersect with the medium, we assume it is out of the medium, so we will just continue with the path
            return this->Li(scene, sampler, ray);
        }
        // if it intersects with the medium, it means we are still inside the medium
        float tMax = itsCheck.t;
        
        // sample the medium
        MediumQueryRecord mRec;
        mRec.p = ray.o;
        itsCheck.medium->sample(mRec, sampler); // sample mRec.p point inside the medium and get its properties
        
        // now I can apply the RTE for this point
        // in-scattering
        //printf("rayMarching: in-scattering\n");
        Color3f Lis(0.0f);  // in-scattering radiance
        PhaseFunctionQueryRecord pRec(ray.d); // we need to pass the direction of the ray
        //printf("rayMarching: sampling phase function\n");
        itsCheck.medium->getPhaseFunction()->sample(pRec, sampler->next2D()); // this will return a new direction for the in-scattered ray
        // decide by russian roulette if the point gets in-scattered or not (the more depth, the less probability)
        //printf("rayMarching: russian roulette\n");
        float survivalProb = std::min(itsCheck.medium->getPhaseFunction()->eval(pRec).getLuminance()/(depth), 0.99f);
        //printf("rayMarching: survivalProb = %f\n", survivalProb);
        if (depth < 3 || sampler->next1D() < survivalProb) {
            Ray3f inScatteringRay(ray.o, pRec.wo);
            Lis = mRec.sigmaS * this->rayMarching(scene, sampler, inScatteringRay, depth+1);
            Lis *= survivalProb;
        }
        // light emission of my point in the medium
        //printf("rayMarching: light emission\n");
        Color3f Le = mRec.sigmaA * mRec.Le;

        // continue with ray marching
        // store the distance to the next medium interaction (random)
        //printf("rayMarching: ray marching\n");
        float t = -std::log(1 - sampler->next1D()) / mRec.sigmaT.getLuminance();
        mRec.t = t > tMax ? tMax : t;   // careful not to go beyond the maximum distance
        Ray3f rayMarchingRay(ray.o + ray.d * mRec.t, ray.d);    // new ray!
        // light loss due to extinction
        Color3f losses = exp(-mRec.sigmaT * t);    // the distance it travels implies a loss of light
        Lo = Le + Lis + losses * this->Li(scene, sampler, rayMarchingRay);
        return Lo;
    }

    // Color3f rayMarching(const Scene* scene, Sampler* sampler, const Ray3f& ray, int depth=0) const {
    //     // starting from the point of intersection with the medium, will march inside the medium until it exits
    //     // or until it gets extinguished by russian roulette
    //     Color3f Lo(0.0f);   // the radiance we will return
    //     // first, get the maximum distance the ray will travel inside the medium
    //     Intersection its;
    //     bool intersects = scene->rayIntersect(ray, its);
    //     if (!intersects || its.medium == nullptr) {
    //         // if the ray doesnt itersect with the medium, we assume it is out of the medium, so we will just continue with the path
    //         return this->Li(scene, sampler, ray);
    //     }
    //     float tMax = its.t;
    //     MediumQueryRecord mRec;
    //     mRec.p = its.p;
    //     bool sampled = its.medium->sampleDistance(mRec, sampler);
    //     if (!sampled || mRec.t >= tMax) { // if not sampled or the sampled distance is greater than the maximum distance, we're out of the medium
    //         return this->Li(scene, sampler, Ray3f(ray.o + ray.d * tMax, ray.d));
    //     }
    //     // if sampled, update the ray
    //     Ray3f rayMarchingRay(ray.o + ray.d * mRec.t, ray.d);
    //     // apply the RTE
    //     // in-scattering will be given by the scattering coefficient (sigmaS) and a new ray (sampled from the phase function) //
    //     // first, sample the phase function
    //     PhaseFunctionQueryRecord pRec(ray.d); // we need to pass the direction of the ray
    //     its.medium->getPhaseFunction()->sample(pRec, sampler->next2D());
    //     Ray3f inScatteringRay(ray.o, pRec.wo);
    //     // decide by russian roulette if the ray gets in-scattered or not
    //     float survivalProb = std::min(its.medium->getPhaseFunction()->eval(pRec).getLuminance()*1/depth, 0.95f);
    //     if (depth < 3 || sampler->next1D() < survivalProb) {
    //         Lo += mRec.sigmaS * this->rayMarching(scene, sampler, inScatteringRay, depth+1);
    //     }
    //     mRec.Le = Color3f(1.0f, 1.0f, 1.0f);
    //     Lo += mRec.sigmaA * mRec.Le;
    //     // out-scattering will be given by the extinction coefficient (sigmaT) and the ray
    //     Lo *= mRec.sigmaT * mRec.t;
    //     return Lo + this->rayMarching(scene, sampler, rayMarchingRay, depth);   // TODO: divide by pdf
    // }

    Color3f pathTracing(const Scene* scene, Sampler* sampler, const Ray3f& ray, const Intersection& its) const {
        Color3f Lo(0.0f);   // the radiance we will return
        Point2f sample = sampler->next2D();
        BSDFQueryRecord bsdfQR(its.toLocal(-ray.d), sample);
        Color3f brdfSample = its.mesh->getBSDF()->sample(bsdfQR, sample);
        if (brdfSample.isZero() || brdfSample.hasNaN()) {   // if it is not valid, return black
            return Lo;
        }
        Ray3f bouncedRay(its.p, its.toWorld(bsdfQR.wo));
        // decide wether to continue or not via russian roulette
        float survivalProb = std::min(brdfSample.maxCoeff(), 0.95f);
        if (sampler->next1D() > survivalProb) {
            return Lo;
        }
        Lo += brdfSample * this->Li(scene, sampler, bouncedRay);
        return Lo;  // TODO: divide by pdf
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Intersection its;
        /* PATH TERMINATION CASES */
        if (!scene->rayIntersect(ray, its)) { // if no intersection, return background color
            return scene->getBackground(ray);
        }
        if (its.mesh->isEmitter()) {    // if the intersection is an emitter, add the contribution
                EmitterQueryRecord emitterQR(its.p);
                emitterQR.n = its.shFrame.n;
                emitterQR.ref = ray.o;
                emitterQR.uv = its.uv;
                emitterQR.wi = ray.d;
                emitterQR.dist = its.t;
                return its.mesh->getEmitter()->eval(emitterQR);
        }

        /* PATH CONTINUATION CASES */
        Color3f Lo(0.0f);   // the radiance we will return
        // intersected with a medium
        if (its.medium != nullptr) {
            // if i hit a medium, will start ray marching inside the medium
            Ray3f mediumRay(its.p, ray.d);  // this ray's origin is the point of intersection with the medium
            Lo = rayMarching(scene, sampler, mediumRay);
        } else {
            // if i hit a normal mesh, will continue the path
            Ray3f pathRay(its.p, ray.d);
            Lo = pathTracing(scene, sampler, pathRay, its);
        }
        return Lo;
    }

    std::string toString() const {
        return "Direct Multiple Importance Sampling []";
    }
};

NORI_REGISTER_CLASS(PathTracingMedia, "path_media");
NORI_NAMESPACE_END