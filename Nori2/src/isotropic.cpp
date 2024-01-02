/**
 * This is a simple implementation of the isotropic phase function.
 * It is used by default by the \ref Medium class.
*/

#include <nori/phase.h>

NORI_NAMESPACE_BEGIN

class Isotropic : public PhaseFunction {
public:
    Isotropic(const PropertyList &propList) {
        /* No parameters this time */
    }

    float sample(const PhaseFunctionQueryRecord &pRec, const Point2f &sample) const {
        return 1.0f;
    }

    float eval(const PhaseFunctionQueryRecord &pRec) const {
        return INV_FOURPI;
    }

    float pdf(const PhaseFunctionQueryRecord &pRec) const {
        return INV_FOURPI;
    }

    std::string toString() const {
        return "IsotropicPhaseFunction[]";
    }
};

NORI_REGISTER_CLASS(Isotropic, "isotropic");

NORI_NAMESPACE_END