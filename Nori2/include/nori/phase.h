#pragma once

#include <nori/object.h>

NORI_NAMESPACE_BEGIN

struct PhaseFunctionQueryRecord {
    Vector3f wi;
    Vector3f wo;
    PhaseFunctionQueryRecord(const Vector3f &wi)
        : wi(wi) { }
    PhaseFunctionQueryRecord(const Vector3f &wi, const Vector3f &wo)
        : wi(wi), wo(wo) { }
};

class PhaseFunction : public NoriObject {
public:
    // PhaseFunction(const NoriObject &noriObject);
    /**
     * \brief Sample the phase function and return the importance weight (i.e. the
     * value of the phase function divided by the probability density of the sample
     * with respect to solid angles).
     * \param pRec
     * A phase function query record
     * \param sample
     * A uniformly distributed sample on \f$[0,1]^2\f$
     * \return
     * The phase function value divided by the probability density of the sample.
    */
    virtual float sample(const PhaseFunctionQueryRecord &pRec, const Point2f &sample) const = 0;

    /**
	 * \brief Evaluate the phase function for a pair of directions 
	 * specified in \code pRec
	 * 
	 * \param pRec
	 *     A record with detailed information on the PHASE query
	 * \return
	 *     The phase function value, evaluated for each color channel
	 */
	virtual float eval(const PhaseFunctionQueryRecord &pRec) const = 0;

	/**
	 * \brief Compute the probability of sampling \c pRec.wo
	 * (conditioned on \c pRec.wi).
	 *
	 * This method provides access to the probability density that
	 * is realized by the \ref sample() method.
	 *
	 * \param pRec
	 *     A record with detailed information on the PHASE query
	 *
	 * \return
	 *     A probability/density value expressed with respect
	 *     to the solid angle measure
	 */

	virtual float pdf(const PhaseFunctionQueryRecord &pRec) const = 0;
	
	/**
	 * \brief Return the type of object (i.e. Mesh/PHASE/etc.) 
	 * provided by this instance
	 * */ 
	EClassType getClassType() const { return EPhaseFunction; }
};

NORI_NAMESPACE_END