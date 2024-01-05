/**
 * This is a simple implementation of the isotropic phase function.
 * It is used by default by the \ref Medium class.
*/

#include <nori/phase.h>
#include <nori/warp.h>

NORI_NAMESPACE_BEGIN

class Isotropic : public PhaseFunction {
public:
	Isotropic(const PropertyList &) {
		/* No parameters */
	}

	/// Evaluate the phase function
	Color3f eval(const PhaseFunctionQueryRecord &) const {
		return INV_FOURPI;
	}

	/// Compute the density of \ref sample() wrt. solid angles
	float pdf(const PhaseFunctionQueryRecord &) const {
		return INV_FOURPI;
	}

	/// Draw a a sample from the phase function
	Color3f sample(PhaseFunctionQueryRecord &pRec, const Point2f &sample) const {
		pRec.wo = Warp::squareToUniformSphere(sample);

		/* eval() / pdf() = 1.0. There
		   is no need to call these functions. */
		return 1.0f;
	}

	/// Return a human-readable summary
	std::string toString() const {
		return std::string("Isotropic[]");
	}

	EClassType getClassType() const { return EPhaseFunction; }
};

NORI_REGISTER_CLASS(Isotropic, "isotropic");

NORI_NAMESPACE_END