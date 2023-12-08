#version 430 core

uniform vec3 cameraPosition;
uniform vec3 backgroundColor;
uniform bool bOverrideColor;
uniform vec3 overrideColor;

out vec4 FragColor;

in vec3 vertexWorldPos;
in vec3 vertexNormal;


#define EPS 0.0001

#define Ka 0.05
#define Kd 0.5
#define shininess 180.
#define eta3dReal vec3(1.5)
#define eta3dImag vec3(0.)

float real_fresnel(in float cos_th, in float eta) {
    float eta_sq = eta*eta;
    float cos_th_sq = cos_th*cos_th;
    float c_sq = eta_sq - 1 + cos_th_sq;
    if (c_sq < 0.) {
        return 1.;
    }
    float c = sqrt(c_sq); // Positive ??

    float f_s = (cos_th-c)/(cos_th+c);
    f_s *= f_s;
    float f_p = (eta_sq*cos_th-c)/(eta_sq*cos_th+c);
    f_p *= f_p;

    return clamp((f_s+f_p)/2., EPS, 1.-EPS);
}

float complex_fresnel(in float cos_th, in vec2 eta) {
    vec2 eta_sq = vec2(eta.x*eta.x-eta.y*eta.y, 2.*eta.x*eta.y);
    float cos_th_sq = cos_th*cos_th;
    vec2 c_sq = vec2(eta_sq.x - 1 + cos_th*cos_th, eta_sq.y);
    vec2 polar_c_sq = vec2(length(c_sq), atan(c_sq));
    vec2 c = sqrt(polar_c_sq.x)*vec2(cos(polar_c_sq.y*.5), sin(polar_c_sq.y*.5));
    vec2 complex_f_s = vec2(
    cos_th_sq-c.x*c.x+c.y*c.y,
    -2.*(c.y*cos_th)
    ) / ((cos_th+c.x)*(cos_th+c.x)+(c.y*c.y));
    float f_s = complex_f_s.x*complex_f_s.x+complex_f_s.y*complex_f_s.y;

    vec2 complex_f_p = vec2(
    eta_sq.x*eta_sq.x*cos_th_sq-c.x*c.x+c.y*c.y-eta_sq.y*eta_sq.y*cos_th_sq,
    +2.*(c.x*eta_sq.y*cos_th - eta_sq.x*cos_th*c.y)
    ) / ((eta_sq.x*cos_th+c.x)*(eta_sq.x*cos_th+c.x)+(eta_sq.y*cos_th+c.y)*(eta_sq.y*cos_th+c.y));
    float f_p = complex_f_p.x*complex_f_p.x+complex_f_p.y*complex_f_p.y;

    return (f_s+f_p)/2.;
}

float fresnel(in float cos_th, in vec2 eta) {
    if (abs(eta.y) < 0.00001) {
        return real_fresnel(cos_th, eta.x);
    }
    return complex_fresnel(cos_th, eta);
}

float MicroFacetDist(float cos_th, float alpha) {
    float pi = 3.141593;
    float xhi = step(0., cos_th);
    float cos_th_sq = cos_th*cos_th;
    float tan_th_sq = 1./cos_th_sq-1.;
    float a = xhi/(pi*cos_th_sq*cos_th_sq);
    float b = alpha/(alpha*alpha+tan_th_sq);
    return a*b*b;
}

float GGXDist(float cos_th, float alpha) {
    float cos_th_sq = cos_th*cos_th;
    float tan_th_sq = 1./cos_th_sq-1.;
    return 1./(cos_th+sqrt(cos_th_sq+alpha*alpha*(1.-cos_th_sq)));
}

float cookTorranceSpecular(in vec4 v, in vec4 n, in vec3 l, in vec2 eta) {
    float alpha = mix(0.5, 0.001, sqrt(shininess*0.005));
    vec3 halfVector = normalize(v.xyz+l);
    float cos_th   = dot(halfVector, l.xyz);
    float cos_th_i = dot(v.xyz, n.xyz);
    float cos_th_o = dot(l.xyz, n.xyz);
    float cos_th_h = dot(halfVector, n.xyz);

    if ((cos_th_o < 0) || (cos_th_i < 0)) {
        return 0.;
    }

    return fresnel(cos_th, eta)*MicroFacetDist(cos_th_h, alpha)*GGXDist(cos_th_i, alpha)*GGXDist(cos_th_o, alpha);
}

vec4 directIllumination(vec4 color, vec4 p, vec4 n, vec4 v, vec3 l)
{
    // Goal: compute direct illumination at point p, from the light source at lightPosition.
    // color: local material color, RGBA
    // p: coordinates of intersection point, xyzw
    // n: normal at intersection point, xyzw
    // v: incoming viewing direction. Points towards the eye.
    // l: point -> light vector

    float lightDist = length(l);
    l /= lightDist;

    if (dot(n.xyz, l) < 0) {
        return vec4(Ka*color.xyz, 1.0);
    }

    float ambt = Ka;
    float diff = Kd*max(0., dot(n.xyz, l));
    vec3 spec;
    spec = vec3(cookTorranceSpecular(v, n, l, vec2(eta3dReal.x, eta3dImag.x)),
    cookTorranceSpecular(v, n, l, vec2(eta3dReal.y, eta3dImag.y)),
    cookTorranceSpecular(v, n, l, vec2(eta3dReal.z, eta3dImag.z)));

    return vec4(((ambt+diff)*color.xyz+spec), 1.0);
}


void main() {
    float fogAttenuation = exp(-0.02*length(cameraPosition-vertexWorldPos));
    vec3 dir = normalize(cameraPosition - vertexWorldPos);

    FragColor = vec4(mix(backgroundColor, directIllumination(vec4(0.9), vec4(vertexWorldPos, 0.), vec4(vertexNormal, 0.), vec4(dir, 0.), vec3(cameraPosition)).xyz, fogAttenuation), 1.);
}