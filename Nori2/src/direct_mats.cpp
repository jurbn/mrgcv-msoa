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
            Intersection its;   
            if (!scene->rayIntersect(recursiveRay, its)){
                return Lo;
            }
            if (its.mesh->isEmitter()) {
                // add the radiance to the output
                EmitterQueryRecord lRecE(its.p);
                Lo += throughput * its.mesh->getEmitter()->eval(lRecE);
            }

            // Sample the BSDF to generate a new ray direction
            BSDFQueryRecord bRec(its.shFrame.toLocal(-recursiveRay.d));
            Color3f bsdf = its.mesh->getBSDF()->sample(bRec, sampler->next2D());   // this is the color of the surface
            float cosTheta = Frame::cosTheta(bRec.wo);

            // I GET LOTS OF ZERO BSDFS HERE;;;; WHYYYYYYY
            if (bsdf.isZero() || cosTheta <= 0.0f)
                return Color3f(1.0f, 0.0f, 1.0f);   //Lo;

            float pdf = its.mesh->getBSDF()->pdf(bRec);
            throughput *= bsdf; // * cosTheta / pdf;

            if (depth > 2) {    // ensure at least 3 bounces
                // apply russian roulette
                float survivalProbability = std::min(0.95f, throughput.maxCoeff());
                if (sampler->next1D() > survivalProbability)
                    return Lo;
                throughput /= survivalProbability;
            }
            recursiveRay = Ray3f(its.p, its.toWorld(bRec.wo));
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