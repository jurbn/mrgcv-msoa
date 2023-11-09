#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectMaterialSampling : public Integrator {
public:
	DirectMaterialSampling(const PropertyList& props) {
		/* No parameters this time */
	}

    // TODO: this breaks on rough surfaces, check it out
    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo = Color3f(0.f); // output radiance
        Color3f throughput = Color3f(1.f); // this is more or less like a weight for the radiance we get from the emitter
        Ray3f recursiveRay = ray;   // this is a copy we can bounce around
        int depth = 1;  // actual number of bounces

        while (true) {  // loop until we dont intersect or russian roulette kills us
            Intersection its;   // intersection point
            if (!scene->rayIntersect(recursiveRay, its)){
                return Lo;
            }
            if (its.mesh->isEmitter()) {
                // add the radiance to the output
                EmitterQueryRecord emQR(its.p);
                emQR.ref = recursiveRay.o;
                emQR.wi = recursiveRay.d;
                emQR.n = its.shFrame.n; 
                Lo += throughput * its.mesh->getEmitter()->eval(emQR);
            }
                    
            if (depth > 2) {    // ensure at least 3 bounces
                // apply russian roulette to decide wether to continue or not
                float survivalProbability = std::min(0.99f, throughput.maxCoeff()); // cap it at 0.99
                if (sampler->next1D() > survivalProbability)
                    return Lo;
                throughput /= survivalProbability;  // throughput gets smaller as we continue (this is the weight of the radiance)
            }
            // sample the BSDF to generate a new ray direction
            BSDFQueryRecord bsdfQR(its.shFrame.toLocal(-recursiveRay.d));
            Color3f bsdf = its.mesh->getBSDF()->sample(bsdfQR, sampler->next2D());   // this is the color of the surface
            throughput *= bsdf; // this way, the next time we bounce, we will have the color of this surface
            recursiveRay = Ray3f(its.p, its.toWorld(bsdfQR.wo));
            depth++;
        }
        return Lo;
    }

    std::string toString() const {
        return "Direct Material Sampling []";
    }
};

NORI_REGISTER_CLASS(DirectMaterialSampling, "direct_mats");
NORI_NAMESPACE_END