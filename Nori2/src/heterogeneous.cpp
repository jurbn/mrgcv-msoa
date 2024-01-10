#include <nori/medium.h>
#include <nori/sampler.h>
#include <nori/density.h>
#include <cmath>

NORI_NAMESPACE_BEGIN

class HeterogeneousMedium : public Medium {
public:
    HeterogeneousMedium(const PropertyList &propList) {
        m_sigmaS = propList.getColor("sigmaS", Color3f(0.0f));
        m_sigmaA = propList.getColor("sigmaA", Color3f(0.0f));
        m_mediumToWorld = propList.getTransform("toWorld", Transform().inverse());
		m_sigmaT = m_sigmaA + m_sigmaS;
		m_phaseFunction = nullptr;
		m_Le = propList.getColor("Le", Color3f(0.0f));
		m_densityFunction = nullptr;
    }

	/**
	 * \brief Sample a point inside the medium
	*/
	void sample(MediumQueryRecord &mRec, Sampler *sampler) const {
		// we will sample a point inside the medium and obtain its properties via the density function
        float sample_value = m_densityFunction->sample(mRec);
		// we will map the sampled value to the medium's properties
		
	}

	// sampleDistance must select a next medium interaction and return the properties of that interaction (the return must be false if out of the medium)
	bool sampleDistance(MediumQueryRecord &mRec, Sampler *sampler) const {
		// to sample the distance to the next medium interaction, we will use delta tracking
		// given the ray:
		float t = -std::log(1 - sampler->next1D()) / m_sigmaT.getLuminance();  // this distance is sampled from an exponential distribution with parameter sigmaT
		// check if the next point of intersection is inside the medium (if not, we're out of the medium)
		sample(mRec, sampler);
		// update the medium query record
		mRec.t = t;
		mRec.pdf = 1.0f;
		return true;
	}

	Color3f evalTransmittance(const Ray3f &ray, Sampler *sampler) const {
		// this will evaluate the transmittance along the path segment defined by the ray
		// this is done using the Radiative Transfer Equation
		// repeat until the ray exits the medium
		// printf("evalTransmittance\n");
		Ray3f mediumRay(ray);	// this ray's origin is the point of intersection with the medium (multiple steps)
		int steps = 0;			// number of steps (debugging)
		// initialize the transmittance
		Color3f transmittance(1.0f);	// this is the light coming from the medium (radiance)
		while (true){
			// 1. sample the distance to the next medium interaction
			MediumQueryRecord mRec;
			bool sampled = sampleDistance(mRec, sampler);
			float t = mRec.t;
			if (!sampled) {	// if the ray doesnt intersect with nothing, we're out of the medium
				printf("evalTransmittance: not sampled with t = %f\n", t);
				break;
			}
			// printf("evalTransmittance: sampled, t = %f\n", t);
			// 2. update the ray (because we are in the medium, the origin of the ray is the point of intersection)
			mediumRay = Ray3f(mediumRay.o + mediumRay.d * t, mediumRay.d);
			std::string rayOrigin = mediumRay.o.toString();
			printf("evalTransmittance: mediumRay.o = %s\n", rayOrigin.c_str());
			//std::cout << "mediumRay.o = " << mediumRay.o << std::endl;
			// create a PhaseFunctionQueryRecord with the sampled direction
			PhaseFunctionQueryRecord pRec(mediumRay.d);
			// 3. update the transmittance by using the Radiative Transfer Equation
			// in-scattering will be given by the scattering coefficient and the phase function
			Color3f inScattering = m_sigmaS * m_phaseFunction->eval(pRec);
			// emission will be given by the absorption coefficient and the phase function
			Color3f emission = m_sigmaA * m_phaseFunction->eval(pRec);
			// update the transmittance
			transmittance *= (inScattering + emission);
			steps++;
		}
		printf("evalTransmittance: transmittance = %s, steps = %d\n", transmittance.toString(), steps);
		return transmittance;
	}

	void addChild(NoriObject *child) {
		if (child->getClassType() == EPhaseFunction) {
			if (m_phaseFunction)
				throw NoriException("Heterogeneous::addChild(): A phase function has already been specified!");
			m_phaseFunction = static_cast<PhaseFunction *>(child);
		} else {
			throw NoriException("Heterogeneous::addChild(): Expected a phase function!");
		}
	}

	void activate() {
		// If no phase function was specified, instantiate an isotropic one
		if (!m_phaseFunction) {
			m_phaseFunction = static_cast<PhaseFunction *>(NoriObjectFactory::createInstance("henyey_greenstein", PropertyList()));
		}
	}

	std::string toString() const {
		return tfm::format(
			"HeterogeneousMedium[\n"
			"  phaseFunction = %s\n"
			"  sigmaS = %1,\n"
			"  sigmaA = %2,\n"
			"]",
			m_phaseFunction ? (m_phaseFunction->toString()) : std::string("null"),
			m_sigmaS.toString(),
			m_sigmaA.toString()
		);
	}

private:
	Color3f m_sigmaS;			// scattering coefficient
	Color3f m_sigmaA;			// absorption coefficient
	Color3f m_sigmaT;			// extinction coefficient
	Transform m_mediumToWorld;	// transform from medium to world space
	Color3f m_Le;				// emission coefficient
	DensityFunction *m_densityFunction;	// the density function of the medium
};

NORI_REGISTER_CLASS(HeterogeneousMedium, "heterogeneous");

NORI_NAMESPACE_END