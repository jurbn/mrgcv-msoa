#include <nori/density.h>
#include <nori/medium.h>

NORI_NAMESPACE_BEGIN

class PerlinNoise : public DensityFunction {
public:
    PerlinNoise(const PropertyList &propList) {
        seed = propList.getInteger("seed", 0);
        octaves = propList.getInteger("octaves", 1);
        persistance = propList.getInteger("persistance", 1);
        frequency = propList.getFloat("frequency", 1.0f);
    }

    virtual ~PerlinNoise() {}

    
    float noise(Point3f p, float frequency) {
        Point3f v = Point3f(p.x() * frequency, p.y() * frequency, p.z() * frequency);

        // Calculate grid cell coordinates
        float ix1 = floor(v.x());
        float iy1 = floor(v.y());
        float iz1 = floor(v.z());
        float ix2 = ix1 + 1.0;
        float iy2 = iy1 + 1.0;
        float iz2 = iz1 + 1.0;

        // Calculate interpolation values for each corner of the cube
        float fx = hermite(fract(v.x()));
        float fy = hermite(fract(v.y()));
        float fz = hermite(fract(v.z()));

        // Get random values at each corner of the cube
        float c000 = rand(Point3f(ix1, iy1, iz1));
        float c100 = rand(Point3f(ix2, iy1, iz1));
        float c010 = rand(Point3f(ix1, iy2, iz1));
        float c110 = rand(Point3f(ix2, iy2, iz1));
        float c001 = rand(Point3f(ix1, iy1, iz2));
        float c101 = rand(Point3f(ix2, iy1, iz2));
        float c011 = rand(Point3f(ix1, iy2, iz2));
        float c111 = rand(Point3f(ix2, iy2, iz2));

        // Interpolate along x-axis
        float x00 = mix(c000, c100, fx);
        float x10 = mix(c010, c110, fx);
        float x01 = mix(c001, c101, fx);
        float x11 = mix(c011, c111, fx);

        // Interpolate along y-axis
        float y0 = mix(x00, x10, fy);
        float y1 = mix(x01, x11, fy);

        // Interpolate along z-axis and return final noise value
        return mix(y0, y1, fz);
    }


    float pnoise(Point3f p, float freq, int steps, float persistence) {
        float value = 0.0;
        float ampl = 1.0;
        float sum = 0.0;
        for(int i=0 ; i<steps ; i++)
        {
            sum += ampl;
            value += noise(p, freq) * ampl;
            freq *= 2.0;
            ampl *= persistence;
        }
        return value / sum;
    }

    /**
     * \brief
     * Sample a point inside the medium
     * \param[out] mRec
     * The medium query record
     * \return
     * The sampled point's properties
    */
    Vector4f sample(MediumQueryRecord &mRec) {
        // get the point in the medium
        Point3f p = mRec.p;
        // get the noise value
        float noiseValue = pnoise(p, frequency, octaves, persistance);

        // Map noise value to color transitions
        // the higher the point, the more transparent it is
        // p.z will be between 1.35 and 1.6, map that to between 0 and 1
        float height = (p.z() - 1.35) / (1.6 - 1.35);
        float zGradient = 1.f-height;
        float zGradientStep = 0.1f;
        // the higher the value on x, the more transparent it is
        float xGradient = abs(p.x());
        // value of y will be between -0.06 and 0.11 (map that to between -1 and 1)
        float yGradient = abs((p.y() + 0.06) / (0.11 + 0.06) * 2.0 - 1.0);


        Vector4f brighterColor = Vector4f(1.0, 0.65, 0.1, 1.0); // 1.0, 0.65, 0.1
        Vector4f darkerColor = Vector4f(1.0, 0.0, 0.15, 0.0); // 1.0, 0.0, 0.15
        Vector4f middleColor = brighterColor.cwiseProduct(darkerColor);

        float firstStep = smoothstep(0.0, noiseValue, zGradient);
        float darkerColorStep = smoothstep(0.0, noiseValue, zGradient - zGradientStep);
        float darkerColorPath = firstStep - darkerColorStep;
        Vector4f color = mix(brighterColor, darkerColor, darkerColorPath);

        float middleColorStep = smoothstep(0.0, noiseValue, zGradient - 0.2 * 2.0);

        color = mix(color, middleColor, darkerColorStep - middleColorStep);
        color = mix(Vector4f(0.0), color, firstStep);
        color = mix(Vector4f(0.0), color, firstStep);
        color = mix(Vector4f(0.0), color, firstStep);

        float peripheralGradient = std::max(xGradient, yGradient);
        // normalize that peripheral gradient value that can go from 0 to 1 to a value that goes from -1 to 1
        peripheralGradient = (peripheralGradient - 0.5) * 2.0;
        // apply sigmoid function to the peripheral gradient
        color *= 1.0 / (exp(-peripheralGradient/0.1) + 1.0);
        
        // apply sigmoid function to the alpha value too (smoothen the edges of the flames)
        color.w() *= 1.0 / (exp(-color.w()/0.2) + 1.0);

        return color;
    }

    std::string toString() const {
        return "PerlinNoise[]";
    }



protected:
    int seed;   // Seed for the random number generator
    int octaves;    // Number of octaves
    float persistance;    // Persistance of the noise
    float frequency;    // Frequency of the noise

    float smoothstep(float edge0, float edge1, float x) {
        // Scale x to the range [0, 1]
        x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0);
        // Smooth interpolation function
        return x * x * (3 - 2 * x);
    }

    float dot (Point3f p1, Point3f p2) {
        return p1.x() * p2.x() + p1.y() * p2.y() + p1.z() * p2.z();
    }

    float rand(Point3f p){
        return fract(sin(dot(p ,Point3f(12.9898,78.233, 45.543))) * 43758.5453);
    }

    float fract(float value) {
        return value - floor(value);
    }

    // Hermite interpolation function
    float hermite(float t) {
        return t * t * (3.0 - 2.0 * t);
    }

    float mix(float x, float y, float a) {
        return x * (1.0 - a) + y * a;
    }

    Vector4f mix(Vector4f x, Vector4f y, float a) {
        return x * (1.0 - a) + y * a;
    }

};

NORI_REGISTER_CLASS(PerlinNoise, "perlin");
NORI_NAMESPACE_END