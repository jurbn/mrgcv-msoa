#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/medium.h>
#include <nori/phase.h>

NORI_NAMESPACE_BEGIN
class PathTracingMedia: public Integrator {
public:
    PathTracingMedia(const PropertyList &props) {
        /* No parameters this time */
    }

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        // this will implement the path tracing algorithm with support for participating media
        Color3f Lo(0.0f);   // the radiance we will return
        int depth = 1;
        float survivalProb;
        Color3f throughput(1.0f);
        Ray3f bouncyRay = ray;
        Intersection its;
        while (true) {
            // get the intersection
            bool intersects = scene->rayIntersect(bouncyRay, its);
            if (!intersects) {
                // if the ray doesnt intersect with nothing, we will add the background color
                // to the radiance we will return
                Color3f backgroundColor = scene->getBackground(bouncyRay);
                Lo += backgroundColor * throughput;
                break;
            }
            // first, check if the ray intersects a medium (intersection's pointer to medium is not null)
            MediumQueryRecord mediumQR;
            if (its.mesh->isMedium()) {
                // Sample medium using delta tracking
                float t;
                Color3f weight;
                std::string mediumSpecs = its.mesh->getMedium()->toString();
                bool sampled = its.mesh->getMedium()->sampleDistance(bouncyRay, sampler, t);
                if (!sampled) {
                    break;
                }
                // update the throughput
                throughput *= weight;
                // update the ray
                bouncyRay = Ray3f(bouncyRay.o + bouncyRay.d * t, bouncyRay.d);
                // update the radiance (this is done using the Radiative Transfer Equation)
                Lo += throughput * its.mesh->getMedium()->evalTransmittance(bouncyRay, sampler);
            }
            // the rest will be done like in the `PathTracing` integrator

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
        return "PathTracingMedia[]";
    }

};

NORI_REGISTER_CLASS(PathTracingMedia, "path_media");
NORI_NAMESPACE_END