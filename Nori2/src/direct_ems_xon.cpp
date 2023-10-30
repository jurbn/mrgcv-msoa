#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <nori/integrator.h>

NORI_NAMESPACE_BEGIN

class DirectEmitterSampling : public Integrator
{
public:
	DirectEmitterSampling(const PropertyList &props)
	{
		/* No parameters this time */
	}
	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const
	{
		Color3f Lo(0.);
		// Find the surface that is visible in the requested direction
		Intersection its;
		if (!scene->rayIntersect(ray, its))
			return scene->getBackground(ray);
		float pdflight;
		EmitterQueryRecord emitterRecord(its.p);
		// Randomly select a light source
		const Emitter *emitter = scene->sampleEmitter(sampler->next1D(), pdflight);
		// Sample the light source (get its radiance and direction)
		Color3f Le = emitter->sample(emitterRecord, sampler->next2D(), 0.);

		// direction from the intersection point to the light source
		Vector3f wi = emitterRecord.wi;
		// shadow ray from the intersection point towards the sample incoming direction wi
		Ray3f shadowRay(its.p, wi);
		// distance to the light source
		shadowRay.maxt = (emitterRecord.p - its.p).norm();
		// check for intersections with scene geometry
		if (!scene->rayIntersect(shadowRay))
		{

			BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d),
									   its.toLocal(wi), its.uv, ESolidAngle);
			// For each light, we accomulate the incident light times the
			// foreshortening times the BSDF term
			if (emitterRecord.pdf * pdflight != 0.0f)
			{
				Lo += Le * its.shFrame.n.dot(emitterRecord.wi) * its.mesh->getBSDF()->eval(bsdfRecord) / (emitterRecord.pdf * pdflight);
			}
			if (its.mesh->isEmitter())
			{
				return emitter->eval(emitterRecord);
			}
		}
		else
		{
			return scene->getBackground(ray);
		}
		return Lo;
	}
	std::string toString() const
	{
		return "Direct Emitter Sampling Integrator []";
	}
};

NORI_REGISTER_CLASS(DirectEmitterSampling, "direct_ems");
NORI_NAMESPACE_END
