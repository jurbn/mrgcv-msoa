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
        Color3f Lo(0.0f);
        Intersection its;
        int depth = 1;
        float survivalProb;
        Color3f throughput(1.0f);
        Ray3f bouncyRay = ray;
        while (true){
            float w_ems = 0.f, w_mats = 0.f;
            float p_em_em = 0.f, p_mat_em = 0.f, p_mat_mat = 0.f, p_em_mat = 0.f;
            if (!scene->rayIntersect(bouncyRay, its)) {
                // no intersection
                Lo = throughput * scene->getBackground(bouncyRay);
                break;
            }
            if (its.mesh->isEmitter()) {
                // intersection with an emitter
                EmitterQueryRecord emitterQR(its.p);
                emitterQR.ref = bouncyRay.o;
                emitterQR.wi = bouncyRay.d;
                emitterQR.n = its.shFrame.n; 
                Lo = throughput * its.mesh->getEmitter()->eval(emitterQR);
                break;
            }
            // If it's not an emitter nor background, we will take both samples and weight them

            /*
            Light importance sampling
            */
            Color3f Les(0.0f);  // this is the contribution of the light importance sampling
            // randomly choose an emitter
            float pdflight;	// this is the probability density of choosing an emitter
            EmitterQueryRecord emitterQR_ls(its.p);	// add intersection point to emitterRecord
            const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);
            // get the radiance of said emitter
            Color3f Lem_ls = em->sample(emitterQR_ls, sampler->next2D(), 0.f);
            // create a shadow ray to see if the point is in shadow
            Ray3f shadowRay(its.p, emitterQR_ls.wi);
            shadowRay.maxt = (emitterQR_ls.p - its.p).norm();
            // check if the ray intersects with an emitter or if the first intersection in shadows
            Intersection its_ls;
            bool inShadow = scene->rayIntersect(shadowRay, its_ls);
            if (!inShadow || its_ls.t >= (emitterQR_ls.dist - Epsilon)){
                BSDFQueryRecord bsdfQR_ls(its.toLocal(-bouncyRay.d), its.toLocal(emitterQR_ls.wi), its.uv, ESolidAngle);
                float denominator = pdflight * emitterQR_ls.pdf;
                if (denominator > Epsilon){	// to avoid division by 0 (resulting in NaNs and anoying warnings)
                    emitterQR_ls.dist = its.t;
                    Color3f bsdf = its.mesh->getBSDF()->eval(bsdfQR_ls);
                    p_em_em = denominator;  // its the
                    p_mat_em = its.mesh->getBSDF()->pdf(bsdfQR_ls);//BRDF pdf
                    if (p_em_em + p_mat_em > Epsilon){ // if you dont enter this, Les will be 0
                        Les = (Lem_ls * its.shFrame.n.dot(emitterQR_ls.wi) * bsdf) / denominator;
                        // compute the weight
                        w_ems = p_em_em / (p_em_em + p_mat_em);
                    }
                }
            }

            // russian roulette to see if the ray dies
            if (depth > 2) {
                survivalProb = std::min(throughput.maxCoeff(), 0.99f);
                if (sampler->next1D() > survivalProb) {
                    break;
                } else {
                    throughput /= survivalProb;
                }
            }

            /*
            BRDF sampling
            */
            Color3f Lbs(0.0f);  // BRDF sampling contribution
            BSDFQueryRecord bsdfQR_bs(its.toLocal(-bouncyRay.d), its.uv);
            Color3f brdfSample = its.mesh->getBSDF()->sample(bsdfQR_bs, sampler->next2D());
            if (!(brdfSample.isZero() || brdfSample.hasNaN())) {    // only enter if sample is valid!
                // generate a new ray with the sampled direction
                Ray3f bsdfRay(its.p, its.toWorld(bsdfQR_bs.wo));
                Intersection its_bs;
                if (!scene->rayIntersect(bsdfRay, its_bs)) {
                    // if the ray doesnt intersect, take the background color
                    Color3f backgroundColor = scene->getBackground(bsdfRay);
                    Lbs = backgroundColor * brdfSample;
                } else {
                    // if the ray intersects with an emitter, take the radiance of the emitter
                    if (its_bs.mesh->isEmitter()) {
                        const Emitter* em_bs = its_bs.mesh->getEmitter();
                        EmitterQueryRecord emitterQR_bs(em_bs, its.p, its_bs.p, its_bs.shFrame.n, its_bs.uv);
                        p_mat_mat = its.mesh->getBSDF()->pdf(bsdfQR_bs);
                        p_em_mat = em_bs->pdf(emitterQR_bs);
                        // i need to convert them to the same space first
                        p_em_mat *= scene->pdfEmitter(em);
                        if (p_em_mat + p_mat_mat > Epsilon){
                            Color3f Lem_bs = em_bs->eval(emitterQR_bs);
                            Lbs = Lem_bs * brdfSample;
                            // compute the weight
                            w_mats = p_mat_mat / (p_em_mat + p_mat_mat);
                        }
                    }
                }
            }

            bouncyRay = Ray3f(its.p, its.toWorld(bsdfQR_bs.wo));
            throughput *= Lbs * w_mats + Les * w_ems;
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