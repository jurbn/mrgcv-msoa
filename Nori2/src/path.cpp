#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracing : public Integrator {
public:
	PathTracing(const PropertyList& props) {
		/* No parameters this time */
	}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo(0.0f);   // the radiance we will return
        int depth = 1;
        float survivalProb;
        Color3f throughput(1.0f);
        Ray3f bouncyRay = ray;
        Intersection its;
        while (true) {
            if (!scene->rayIntersect(bouncyRay, its)) {
                // if the ray doesnt intersect with nothing, we will add the background color
                // to the radiance we will return
                Color3f backgroundColor = scene->getBackground(bouncyRay);
                Lo += backgroundColor * throughput;
                break;
            }
            if (its.mesh->isEmitter()) {
                // if the ray intersects with an emitter, we will add the radiance of the emitter
                // to the radiance we will return
                EmitterQueryRecord emitterQR(its.p);
                emitterQR.ref = bouncyRay.o;
                emitterQR.wi = bouncyRay.d;
                emitterQR.n = its.shFrame.n;
                Lo += its.mesh->getEmitter()->eval(emitterQR) * throughput;
                break;
            }
            // if the ray intersects with a surface, we will sample the brdf
            Point2f sample = sampler->next2D();
            BSDFQueryRecord bsdfQR(its.toLocal(-bouncyRay.d), sample);
            Color3f brdfSample = its.mesh->getBSDF()->sample(bsdfQR, sample);
            // check if the brdf sample is valid (absorbed or invalid samples are not valid)
            if (brdfSample.isZero() || brdfSample.hasNaN()) {   // if it is not valid, return black
                break;
            }
            // now create a new ray with the sampled direction
            bouncyRay = Ray3f(its.p, its.toWorld(bsdfQR.wo));
            throughput *= brdfSample;
            if (depth > 2) {    // we want to ensure that the path has at least  bounces
                // start the russian roulette
                // max component of the throughput will be the probability of survival (we cap it at 0.95)
                survivalProb = std::min(throughput.maxCoeff(), 0.95f);
                if (sampler->next1D() > survivalProb) { // this is the russian roulette
                    break;  // if the ray dies, we stop the loop
                } else {
                    throughput /= survivalProb; // if the ray survives, we need to update the throughput
                }
            }
            depth++;
        }
        return Lo;
    }

    std::string toString() const {
        return "Path Tracing []";
    }
};

NORI_REGISTER_CLASS(PathTracing, "path");
NORI_NAMESPACE_END