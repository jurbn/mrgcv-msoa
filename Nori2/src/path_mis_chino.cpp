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
        Color3f throughput(1.0f);
        Ray3f bouncyRay = ray;
        float survivalProb;
        float w_mat = 1.0f, w_em = 0.0f;
        int depth = 1;
        Intersection its;
        if (!scene->rayIntersect(bouncyRay, its)) { // if no intersection, return background color
            return scene->getBackground(bouncyRay);
        }
        while (true) {
            if (its.mesh->isEmitter()) {    // if the intersection is an emitter, add the contribution
                EmitterQueryRecord emitterQR(its.p);
                emitterQR.n = its.shFrame.n;
                emitterQR.ref = bouncyRay.o;
                emitterQR.uv = its.uv;
                Lo += w_mat * throughput * its.mesh->getEmitter()->eval(emitterQR);
                break;
            }
            float pdf_emitter;  // probability density of choosing the to-be-sampled emitter
            const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdf_emitter);
            EmitterQueryRecord emitterQR(its.p);
            emitterQR.uv = its.uv;
            Color3f Le = em->sample(emitterQR, sampler->next2D(), 0.0f);
            Ray3f shadowRay(its.p, emitterQR.wi);
            shadowRay.maxt = (emitterQR.p - its.p).norm();
            Intersection shadowIts;
            bool inShadow = scene->rayIntersect(shadowRay, shadowIts);
            if (!inShadow || (shadowIts.t >= (emitterQR.dist - Epsilon))) {
                BSDFQueryRecord bsdfQR(its.toLocal(-bouncyRay.d), its.toLocal(emitterQR.wi), its.uv, ESolidAngle);
                float ls_den = pdf_emitter * emitterQR.pdf;
                if (ls_den > Epsilon) {
                    Color3f bsdf = its.mesh->getBSDF()->eval(bsdfQR);
                    float pdf_bsdf = its.mesh->getBSDF()->pdf(bsdfQR);
                    float w_em_den = pdf_emitter + pdf_bsdf;
                    if (w_em_den > Epsilon) {
                        w_em = pdf_emitter / w_em_den;
                    } else {
                        w_em = pdf_emitter;
                    }
                    Color3f L_ls = (Le * its.shFrame.n.dot(emitterQR.wi) * bsdf) / ls_den;
                    Lo += w_em * throughput * L_ls;
                }
            }
            // now russian roulette!!!
            if (depth > 2) {
                survivalProb = std::min(throughput.maxCoeff(), 0.95f);
                if (sampler->next1D() > survivalProb) {
                    break;
                }
                throughput /= survivalProb;
            }
            // if no bsdf, new ray!
            BSDFQueryRecord bsdfQR(its.toLocal(-bouncyRay.d));
            Color3f bsdf = its.mesh->getBSDF()->sample(bsdfQR, sampler->next2D());
            if (bsdf.isZero() || bsdf.hasNaN()) {   // if ray gets absorbed, stop!
                break;
            }
            throughput *= bsdf;
            bouncyRay = Ray3f(its.p, its.toWorld(bsdfQR.wo));   // new ray direction
            float pdf_bsdf = its.mesh->getBSDF()->pdf(bsdfQR);
            Point3f origin = its.p;
            if (!scene->rayIntersect(bouncyRay, its)) {
                Lo += w_mat * throughput * scene->getBackground(bouncyRay);
            }
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord emitterQR_new(its.p);
                emitterQR_new.n = its.shFrame.n;
                emitterQR_new.ref = bouncyRay.o;
                emitterQR_new.uv = its.uv;
                float new_pdf_emitter = its.mesh->getEmitter()->pdf(emitterQR_new);
                float w_mat_den = pdf_bsdf + new_pdf_emitter;
                if (w_mat_den > Epsilon) {
                    w_mat = pdf_bsdf / w_mat_den;
                } else {
                    w_mat = pdf_bsdf;
                }
            }
            if (bsdfQR.measure == ESolidAngle) {
                w_mat = 1.0f;
            }
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