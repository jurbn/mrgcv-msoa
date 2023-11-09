/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
	
	v1 - Dec 01 2020
    v2 - Oct 30 2021
	Copyright (c) 2021 by Adrian Jarabo

    Nori is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License Version 3
    as published by the Free Software Foundation.

    Nori is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

#include <nori/bsdf.h>
#include <nori/frame.h>
#include <nori/warp.h>
#include <nori/reflectance.h>
#include <nori/texture.h>

NORI_NAMESPACE_BEGIN

#define KS_THRES 0.

class RoughConductor : public BSDF {
public:
    RoughConductor(const PropertyList& propList) {
        /* RMS surface roughness */
        m_alpha = new ConstantSpectrumTexture(propList.getFloat("alpha", 0.1f));

        /* Reflectance at direction of normal incidence.
           To be used when defining the Fresnel term using the Schlick's approximation*/
        m_R0 = new ConstantSpectrumTexture(propList.getColor("R0", Color3f(0.5f)));
    }


    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
        is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);
        // throw NoriException("RoughConductor::eval() is not yet implemented!");

        // Ro is the reflection at normal incidence!!!!
        // alpha param is defining the roughness of the surface, we need it to compute the beckmann term
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float alpha = m_alpha->eval(bRec.uv).getLuminance();
        float beckmann_term = Reflectance::BeckmannNDF(wh, alpha);
        Color3f fresnel_term = Reflectance::fresnel(bRec.wi.dot(wh), m_R0->eval(bRec.uv));

        // G models the Beckmann’s shadowing-masking term under the Smith approximation,
        // defined as G(ωi,ωo,ωh) =G1(ωi,ωh) G1(ωo,ωh)
        float g1_input = Reflectance::G1(bRec.wi, wh, alpha);
        float g1_output = Reflectance::G1(bRec.wo, wh, alpha);
        float g_term = g1_input * g1_output;

        // implement fr(ωi,ωo) = D(ωh)F((ωh ·ωi),R0)G(ωi,ωo,ωh) / 4cosθicosθo
        return (fresnel_term * beckmann_term * g_term) / (4.0f * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo));
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
        is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;
        // throw NoriException("RoughConductor::eval() is not yet implemented!");

        float pdf = Warp::squareToBeckmannPdf(bRec.wi, m_alpha->eval(bRec.uv).getLuminance());
		return pdf;
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord& bRec, const Point2f& _sample) const {
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        if (Frame::cosTheta(bRec.wi) <= 0){
            return Color3f(0.0f);
        }
        bRec.measure = ESolidAngle;
        // throw NoriException("RoughConductor::sample() is not yet implemented!");

        // sample a direction on the unit sphere using the warp function and set the direction in the emitter query record
		bRec.wo = Warp::squareToBeckmann(_sample, m_alpha->eval(bRec.uv).getLuminance());;
		// compute the radiance
		Color3f radiance = eval(bRec);
		return radiance;
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case ETexture:
            if (name == "R0")
            {
                delete m_R0;
                m_R0 = static_cast<Texture*>(obj);
            }
            else if (name == "alpha")
            {
                delete m_alpha;
                m_alpha = static_cast<Texture*>(obj);
            }
            else
                throw NoriException("RoughConductor::addChild(<%s>,%s) is not supported!",
                    classTypeName(obj->getClassType()), name);
            break;
        default:
            throw NoriException("RoughConductor::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "RoughConductor[\n"
            "  alpha = %f,\n"
            "  R0 = %s,\n"
            "]",
            m_alpha->toString(),
            m_R0->toString()
        );
    }
private:
    Texture* m_alpha;
    Texture* m_R0;
};


class RoughDielectric : public BSDF {
public:
    RoughDielectric(const PropertyList& propList) {
        /* RMS surface roughness */
        m_alpha = new ConstantSpectrumTexture(propList.getFloat("alpha", 0.1f));

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Tint of the glass, modeling its color */
        m_ka = new ConstantSpectrumTexture(propList.getColor("ka", Color3f(1.f)));
    }


    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BSDF -- return zero if the measure is wrong */
        if (bRec.measure != ESolidAngle)
            return Color3f(0.0f);


        throw NoriException("RoughDielectric::eval() is not yet implemented!");
    }

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord& bRec) const {
        /* This is a smooth BSDF -- return zero if the measure is wrong */
        if (bRec.measure != ESolidAngle)
            return 0.0f;

        throw NoriException("RoughDielectric::eval() is not yet implemented!");
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord& bRec, const Point2f& _sample) const {
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        bRec.measure = ESolidAngle;

        throw NoriException("RoughDielectric::sample() is not yet implemented!");
    }

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case ETexture:
            if (name == "m_ka")
            {
                delete m_ka;
                m_ka = static_cast<Texture*>(obj);
            }
            else if (name == "alpha")
            {
                delete m_alpha;
                m_alpha = static_cast<Texture*>(obj);
            }
            else
                throw NoriException("RoughDielectric::addChild(<%s>,%s) is not supported!",
                    classTypeName(obj->getClassType()), name);
            break;
        default:
            throw NoriException("RoughDielectric::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "RoughDielectric[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  ka = %s,\n"
            "]",
            m_alpha->toString(),
            m_intIOR,
            m_extIOR,
            m_ka->toString()
        );
    }
private:
    float m_intIOR, m_extIOR;
    Texture* m_alpha;
    Texture* m_ka;
};



class RoughSubstrate : public BSDF {
public:
    RoughSubstrate(const PropertyList &propList) {
        /* RMS surface roughness */
        m_alpha = new ConstantSpectrumTexture(propList.getFloat("alpha", 0.1f));

        /* Interior IOR (default: BK7 borosilicate optical glass) */
        m_intIOR = propList.getFloat("intIOR", 1.5046f);

        /* Exterior IOR (default: air) */
        m_extIOR = propList.getFloat("extIOR", 1.000277f);

        /* Albedo of the diffuse base material (a.k.a "kd") */
        m_kd = new ConstantSpectrumTexture(propList.getColor("kd", Color3f(0.5f)));
    }


    /// Evaluate the BRDF for the given pair of directions
    Color3f eval(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
        is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return Color3f(0.0f);
		// throw NoriException("RoughSubstrate::eval() is not yet implemented!");

        // CALCULATE F_DIFF
        float f_diff = ((28*m_kd->eval(bRec.uv).getLuminance()) / (23*M_PI));
        f_diff *= (1-std::pow((m_extIOR - m_intIOR) / (m_extIOR + m_intIOR), 2));
        f_diff *= (1-std::pow(1 - 0.5*Frame::cosTheta(bRec.wi), 5));
        f_diff *= (1-std::pow(1 - 0.5*Frame::cosTheta(bRec.wo), 5));

        // CALCULATE F_MF
        Vector3f wh = (bRec.wi + bRec.wo).normalized();
        float alpha = m_alpha->eval(bRec.uv).getLuminance();
        float beckmann_term = Reflectance::BeckmannNDF(wh, alpha);
        float fresnel_term = Reflectance::fresnel(bRec.wi.dot(wh), m_extIOR, m_intIOR);
        // G models the Beckmann’s shadowing-masking term under the Smith approximation,
        // defined as G(ωi,ωo,ωh) =G1(ωi,ωh) G1(ωo,ωh)
        float g1_input = Reflectance::G1(bRec.wi, wh, alpha);
        float g1_output = Reflectance::G1(bRec.wo, wh, alpha);
        float g_term = g1_input * g1_output;
        // implement fr(ωi,ωo) = D(ωh)F((ωh ·ωi),R0)G(ωi,ωo,ωh) / 4cosθicosθo
        float f_mf = (fresnel_term * beckmann_term * g_term) / (4 * Frame::cosTheta(bRec.wi) * Frame::cosTheta(bRec.wo));

        // implement fr(ωi,ωo) = fdiff(ωi,ωo) + fmf(ωi,ωo)
        return f_diff + f_mf;
	}

    /// Evaluate the sampling density of \ref sample() wrt. solid angles
    float pdf(const BSDFQueryRecord &bRec) const {
        /* This is a smooth BRDF -- return zero if the measure
       is wrong, or when queried for illumination on the backside */
        if (bRec.measure != ESolidAngle
            || Frame::cosTheta(bRec.wi) <= 0
            || Frame::cosTheta(bRec.wo) <= 0)
            return 0.0f;

		// throw NoriException("RoughSubstrate::eval() is not yet implemented!");

        // russian roulette between diffuse and microfacet
        float random_number = rand() / (float)RAND_MAX;     // FIXME: this could be done with sampleReuse
        Color3f fresnel = Reflectance::fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);
        float p_microfacet = fresnel.getLuminance();
        float p_diffuse = 1 - p_microfacet;
        if (random_number < p_diffuse){ // this means we sample the diffuse part
            return Warp::squareToBeckmannPdf(bRec.wi, m_alpha->eval(bRec.uv).getLuminance());
        }else{  // this means we sample the microfacet part
            return Warp::squareToCosineHemispherePdf(bRec.wo);
        }
    }

    /// Sample the BRDF
    Color3f sample(BSDFQueryRecord &bRec, const Point2f &_sample) const {
        // Note: Once you have implemented the part that computes the scattered
        // direction, the last part of this function should simply return the
        // BRDF value divided by the solid angle density and multiplied by the
        // cosine factor from the reflection equation, i.e.
        // return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);
        if (Frame::cosTheta(bRec.wi) <= 0)
            return Color3f(0.0f);

        bRec.measure = ESolidAngle;
		// throw NoriException("RoughSubstrate::sample() is not yet implemented!");
        
        // russian roulette between diffuse and microfacet
        float random_number = rand() / (float)RAND_MAX;     // FIXME: this could be done with sampleReuse
        Color3f fresnel = Reflectance::fresnel(Frame::cosTheta(bRec.wi), m_extIOR, m_intIOR);
        float p_microfacet = fresnel.getLuminance();
        float p_diffuse = 1 - p_microfacet;
        if (random_number < p_diffuse){ // this means we sample the diffuse part
            bRec.wo = Warp::squareToBeckmann(_sample, m_alpha->eval(bRec.uv).getLuminance());
        }else{  // this means we sample the microfacet part
            bRec.wo = Warp::squareToCosineHemisphere(_sample);
        }
        return eval(bRec) * Frame::cosTheta(bRec.wo) / pdf(bRec);   // FIXME: not sure if this is correct!!
	}

    bool isDiffuse() const {
        /* While microfacet BRDFs are not perfectly diffuse, they can be
           handled by sampling techniques for diffuse/non-specular materials,
           hence we return true here */
        return true;
    }

    void addChild(NoriObject* obj, const std::string& name = "none") {
        switch (obj->getClassType()) {
        case ETexture:
            if (name == "kd")
            {
                delete m_kd;
                m_kd = static_cast<Texture*>(obj);
            }
            else if (name == "alpha")
            {
                delete m_alpha;
                m_alpha = static_cast<Texture*>(obj);
            }
            else 
                throw NoriException("RoughSubstrate::addChild(<%s>,%s) is not supported!",
                    classTypeName(obj->getClassType()), name);
            break;
        default:
            throw NoriException("RoughSubstrate::addChild(<%s>) is not supported!",
                classTypeName(obj->getClassType()));
        }
    }

    std::string toString() const {
        return tfm::format(
            "RoughSubstrate[\n"
            "  alpha = %f,\n"
            "  intIOR = %f,\n"
            "  extIOR = %f,\n"
            "  kd = %s,\n"
            "]",
            m_alpha->toString(),
            m_intIOR,
            m_extIOR,
            m_kd->toString()
        );
    }
private:
    float m_intIOR, m_extIOR;
    Texture* m_alpha;
    Texture* m_kd;
};

NORI_REGISTER_CLASS(RoughConductor, "roughconductor");
NORI_REGISTER_CLASS(RoughDielectric, "roughdielectric");
NORI_REGISTER_CLASS(RoughSubstrate, "roughsubstrate");

NORI_NAMESPACE_END
