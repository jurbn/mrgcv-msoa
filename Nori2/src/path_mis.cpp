#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathTracingMIS : public Integrator {
public:
	PathTracingMIS(const PropertyList& props) {
		/* No parameters this time */
	}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
        Color3f Lo(0.0f);   // the radiance we will return
        int depth = 1;
        float survivalProb;
        Color3f throughput(1.0f);
        Ray3f bouncyRay = ray;
        Intersection its;
        Color3f L_ls(0.0f); // light sampling contribution to the bounce
        while (true) {
            float w_mats = 0.0f, w_lights = 0.0f; // weights for the light sampling and the brdf sampling
            if (!scene->rayIntersect(bouncyRay, its)) {
                // if the ray doesnt intersect with nothing, we will add the background color
                // to the radiance we will return
                Color3f backgroundColor = scene->getBackground(bouncyRay);
                Lo = backgroundColor * throughput;
                break;
            }
            if (its.mesh->isEmitter()) {
                // if the ray intersects with an emitter, we will add the radiance of the emitter
                // to the radiance we will return
                EmitterQueryRecord emitterQR(its.p);
                emitterQR.ref = bouncyRay.o;
                emitterQR.wi = bouncyRay.d;
                emitterQR.n = its.shFrame.n;
                Lo = its.mesh->getEmitter()->eval(emitterQR) * throughput;
                break;
            }
            // NOW IF THE RAY INTERSECTS WITH A NON-EMITTING SURFACE
            /* LIGHT SAMPLING */
            // randomly choose an emitter and add its contribution to the throughput
            float pdflight;	// this is the probability density of choosing a light source
            EmitterQueryRecord emitterQR_ls(its.p);	// add intersection point to emitterRecord
            const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight); 		// sample a random light source
            Color3f Le = em->sample(emitterQR_ls, sampler->next2D(), 0.);	// radiance of the light source
            Ray3f shadowRay(its.p, emitterQR_ls.wi); // shadow ray that goes from the intersection point to the light source
            shadowRay.maxt = (emitterQR_ls.p - its.p).norm();	// maxt is the distance between the intersection point and the light source (?)
            // if the shadow ray doesnt intersect with the scene, or if it intersects after the light source, then the point is not in shadow
            float p_em_em = 0.f, p_mat_em = 0.f;
            L_ls = Color3f(0.0f);
            Intersection shadowIts;
            bool inShadow = scene->rayIntersect(shadowRay, shadowIts);
            if (!inShadow || (shadowIts.t >= (emitterQR_ls.dist - Epsilon))) {
                BSDFQueryRecord bsdfQR_ls(its.toLocal(-bouncyRay.d), its.toLocal(emitterQR_ls.wi), its.uv, ESolidAngle);
                float denominator = pdflight * emitterQR_ls.pdf;
                if (denominator > Epsilon){	// to avoid division by 0 (resulting in NaNs and anoying warnings)
                    // emitterQR_ls.dist = its.t;
                    Color3f bsdf = its.mesh->getBSDF()->eval(bsdfQR_ls);
                    // update the color
                    L_ls = (Le * its.shFrame.n.dot(emitterQR_ls.wi) * bsdf) / denominator;
                }
                p_em_em = pdflight * emitterQR_ls.pdf;
                p_mat_em = its.mesh->getBSDF()->pdf(bsdfQR_ls);
            }

            if (depth > 2) {    // we want to ensure that the path has at least  bounces
                // start the russian roulette
                // max component of the throughput will be the probability of survival (we cap it at 0.95)
                survivalProb = std::min(throughput.maxCoeff(), 0.99f);
                if (sampler->next1D() > survivalProb) { // this is the russian roulette
                    break;  // if the ray dies, we stop the loop
                } else {
                    throughput /= survivalProb; // if the ray survives, we need to update the throughput
                }
            }

            /* BSDF SAMPLING */
            // if the ray intersects with a surface, we will sample the brdf
            Point2f sample = sampler->next2D();
            BSDFQueryRecord bsdfQR(its.toLocal(-bouncyRay.d), sample);
            Color3f brdfSample = its.mesh->getBSDF()->sample(bsdfQR, sample);
            // UPDATE THE RAY
            bouncyRay = Ray3f(its.p, its.toWorld(bsdfQR.wo));
            // check if the brdf sample is valid (absorbed or invalid samples are not valid)
            if (brdfSample.isZero() || brdfSample.hasNaN()) {   // if it is not valid, color absorbed -> stop the loop
                break;
            }
            if (bsdfQR.measure == EDiscrete) {
                w_mats = 1.0f;
                w_lights = 0.0f;
            } else {
                float p_mat_mat = its.mesh->getBSDF()->pdf(bsdfQR);
                float p_em_mat = em->pdf(emitterQR_ls);
                if (p_em_mat + p_mat_mat > Epsilon) {
                    w_mats = p_mat_mat / (p_em_mat + p_mat_mat);
                }
                if (p_em_em + p_mat_em > Epsilon) {
                    w_lights = p_em_em / (p_em_em + p_mat_em);
                }
            }
            throughput *= brdfSample * w_mats + L_ls * w_lights;
            depth++;
        }
        return Lo;
    }

    std::string toString() const {
        return "Direct Multiple Importance Sampling []";
    }
};

NORI_REGISTER_CLASS(PathTracingMIS, "path_mis");
NORI_NAMESPACE_END