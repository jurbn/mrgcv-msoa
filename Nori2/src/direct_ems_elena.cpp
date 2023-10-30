#include <nori/warp.h>
#include <nori/integrator.h>
#include <nori/proplist.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>

NORI_NAMESPACE_BEGIN

class DirectEmitterSampling : public Integrator {
public :

	DirectEmitterSampling(const PropertyList &props) {
		// No parameters this time
	}
	
	/// Compute the radiance value for a given ray. Just return green here
	Color3f Li(const Scene *scene, Sampler *sampler, const Ray3f &ray) const {
		Color3f Lo(0.);
		// Find the surface that is visible in the requested direction
		Intersection its;
		if(!scene->rayIntersect(ray,its))
			return scene->getBackground(ray);
			
		float pdflight;
    
		EmitterQueryRecord emitterRecord(its.p);
		if (its.mesh->isEmitter()){
			emitterRecord.wi = -ray.d;
			emitterRecord.n = its.shFrame.n;
			emitterRecord.uv = its.uv;
			emitterRecord.ref = ray.o;

			return its.mesh->getEmitter()->eval(emitterRecord);
		}
		
		// Let's choose a numeber between all the light points:

        //First randomly sample a light source in the scene using Scene::sampleEmitter
        const Emitter* em = scene->sampleEmitter(sampler->next1D(), pdflight);
        //then, sample the light source to obtain the sampled EmitterQueryRecord using Emitter::sample
        Color3f Le = em->sample(emitterRecord, sampler->next2D(), 0.);

        Vector3f wi = emitterRecord.wi;
        Ray3f shadowRay(its.p, wi, 0.0001,emitterRecord.dist );
        if (!scene->rayIntersect(shadowRay, its)) {
            BSDFQueryRecord bsdfRecord(its.toLocal(-ray.d), its.toLocal(emitterRecord.wi), its.uv, ESolidAngle);
            Lo += Le * its.shFrame.n.dot(emitterRecord.wi) * its.mesh->getBSDF()->eval(bsdfRecord)/(pdflight*emitterRecord.pdf);
        }		
		return Lo;		
	}
	
	/// Return a human-readable description for debugging purposes
	std::string toString ( ) const {
		/*return tfm::format(
		"NormalIntegrator[\n"
		" myProperty = \"%s\"\n"
		"]" ,
		m_myProperty
		);*/
		return "DirectWhittedIntegrator[]";
	}
	
protected :
	std::string m_myProperty;
};

NORI_REGISTER_CLASS (DirectEmitterSampling , "direct_ems" );
NORI_NAMESPACE_END