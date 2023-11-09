#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class DirectEmitterSampling : public Integrator {
public:
	DirectEmitterSampling(const PropertyList& props) {
		/* No parameters this time */
	}

	Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const {
		Color3f Lo(0.);	// default output value
		Intersection its;	// intersection point
		Intersection shadow_its;	// intersection point for shadow ray
		if (!scene->rayIntersect(ray, its))	// if ray doesnt intersect with scene, assume its background
			return scene->getBackground(ray);
		float pdflight;	// this is the probability density of choosing a light source
		EmitterQueryRecord emitterRecord(its.p);	// add intersection point to emitterRecord
		const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight); 		// sample a random light source
		Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.);	// radiance of the light source
		Ray3f shadowRay(its.p, emitterRecord.wi); // shadow ray that goes from the intersection point to the light source
		shadowRay.maxt = (emitterRecord.p - its.p).norm();	// maxt is the distance between the intersection point and the light source (?)

		if (its.mesh->isEmitter()) {	// if the intersection point is an emittter, output the radiance of the emitter
			// update emitterRecord with the intersection point and the direction of the ray
			emitterRecord.ref = ray.o;
			emitterRecord.wi = ray.d;
			emitterRecord.n = its.shFrame.n; 
			return its.mesh->getEmitter()->eval(emitterRecord); 
		}

		// if the shadow ray doesnt intersect with the scene, or if it intersects after the light source, then the point is not in shadow
		if (!scene->rayIntersect(shadowRay, shadow_its) || !(shadow_its.t < emitterRecord.dist)) {
			Lo = Color3f(0.);	// if the shadow ray intersects with the scene, the output is 0 cause its in shadow
			BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
            float denominator = pdflight * emitterRecord.pdf;
            if (denominator > 0.0f){	// to avoid division by 0 (resulting in NaNs and anoying warnings)
				Color3f bsdf = its.mesh->getBSDF()->eval(bsdfRecord);
				// if (bsdf.isZero())	// sanity check, if bsdf is black, return fuchsia just to know
				// 	Lo = Color3f(1.f, 0.f, 1.f);
			    Lo = (Le * its.shFrame.n.dot(emitterRecord.wi) * bsdf) / denominator;
			}
		}
		return Lo;
	}

	std::string toString() const {
		return "Direct Emitter Sampling []";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END