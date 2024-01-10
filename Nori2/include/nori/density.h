#pragma once

#include <nori/object.h>
#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class DensityFunction : public NoriObject {
public:
    DensityFunction() {}

    DensityFunction(const PropertyList &propList) {
        /* No parameters this time */
    }

    virtual ~DensityFunction() {}

    virtual float sample (MediumQueryRecord &mRec) = 0;

    /**
     * \brief Return the type of object (i.e. Medium/Homogeneous/etc.)
     * provided by this instance
     * */
    EClassType getClassType() const { return EDensityFunction; }

protected:
};

NORI_NAMESPACE_END