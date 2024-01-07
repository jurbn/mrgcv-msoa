#pragma once

#include <nori/object.h>
#include <medium.h>

NORI_NAMESPACE_BEGIN

class DensityFunction : public NoriObject {
public:

    DensityFunction(const PropertyList &propList) {
        /* No parameters this time */
    }

    virtual ~DensityFunction() {}

    /**
     * \brief
     * Sample a point inside the medium
     * \param[out] mRec
     * The medium query record
     * \return
     * The sampled point's properties
    */
    virtual void sample(MediumQueryRecord &mRec) const = 0;

protected:
};

NORI_NAMESPACE_END