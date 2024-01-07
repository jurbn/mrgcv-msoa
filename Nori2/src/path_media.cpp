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

    Color3f rayMarching(const Scene* scene, Sampler* sampler, const Ray3f& ray, int depth=0) const {
        // starting from the point of intersection with the medium, will march inside the medium until it exits
        // or until it gets extinguished by russian roulette
        Color3f Lo(0.0f);   // the radiance we will return
        // first, get the maximum distance the ray will travel inside the medium
        Intersection its;
        bool intersects = scene->rayIntersect(ray, its);
        if (!intersects || its.medium == nullptr) {
            // if the ray doesnt itersect with the medium, we assume it is out of the medium, so we will just continue with the path
            return this->Li(scene, sampler, ray);
        }
        float tMax = its.t;
        MediumQueryRecord mRec;
        mRec.p = its.p;
        bool sampled = its.medium->sampleDistance(mRec, sampler);
        if (!sampled || mRec.t >= tMax) { // if not sampled or the sampled distance is greater than the maximum distance, we're out of the medium
            return this->Li(scene, sampler, Ray3f(ray.o + ray.d * tMax, ray.d));
        }
        // if sampled, update the ray
        Ray3f rayMarchingRay(ray.o + ray.d * mRec.t, ray.d);
        // apply the RTE
        // in-scattering will be given by the scattering coefficient (sigmaS) and a new ray (sampled from the phase function) //
        // first, sample the phase function
        PhaseFunctionQueryRecord pRec(ray.d); // we need to pass the direction of the ray
        its.medium->getPhaseFunction()->sample(pRec, sampler->next2D());
        Ray3f inScatteringRay(ray.o, pRec.wo);
        // decide by russian roulette if the ray gets in-scattered or not
        float survivalProb = std::min(its.medium->getPhaseFunction()->eval(pRec).getLuminance()*1/depth, 0.95f);
        if (depth < 3 || sampler->next1D() < survivalProb) {
            Lo += mRec.sigmaS * this->rayMarching(scene, sampler, inScatteringRay, depth+1);
        }
        mRec.Le = Color3f(1.0f, 1.0f, 1.0f);
        Lo += mRec.sigmaA * mRec.Le;
        // out-scattering will be given by the extinction coefficient (sigmaT) and the ray
        Lo *= mRec.sigmaT * mRec.t;
        return Lo + this->rayMarching(scene, sampler, rayMarchingRay, depth);
    }

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
        return Lo;
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
            return Lo;
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