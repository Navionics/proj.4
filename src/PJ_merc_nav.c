#define PJ_LIB__
#include <proj.h>
#include "projects.h"

PROJ_HEAD(merc_nav, "Mercator Navionics") "\n\tCyl, Sph&Ell\n\tlat_ts=";

#define EPS10 1.e-10
#define KC 1.00676429271698

static XY e_forward (LP lp, PJ *P) {          /* Ellipsoidal, forward */
    XY xy = {0.0,0.0};
    if (fabs(fabs(lp.phi) - M_HALFPI) <= EPS10) {
        proj_errno_set(P, PJD_ERR_TOLERANCE_CONDITION);
        return xy;
    }
    xy.x = P->k0 * lp.lam;
    xy.y = log(tan(.5 * atan(tan(lp.phi) / KC) + M_FORTPI));
    return xy;
}


// static XY s_forward (LP lp, PJ *P) {           /* Spheroidal, forward */
//     XY xy = {0.0,0.0};
//     if (fabs(fabs(lp.phi) - M_HALFPI) <= EPS10) {
//         proj_errno_set(P, PJD_ERR_TOLERANCE_CONDITION);
//         return xy;
// }
//     xy.x = P->k0 * lp.lam;
//     xy.y = P->k0 * log(tan(M_FORTPI + .5 * lp.phi));
//     return xy;
// }


static LP e_inverse (XY xy, PJ *P) {          /* Ellipsoidal, inverse */
    LP lp = {0.0,0.0};
    if ((lp.phi = pj_phi2(P->ctx, exp(- xy.y / P->k0), P->e)) == HUGE_VAL) {
        proj_errno_set(P, PJD_ERR_TOLERANCE_CONDITION);
        return lp;
}
    lp.phi = atan2((tan((atan(exp(xy.y)) * 2.) - M_HALFPI) * KC), 1);
    lp.lam = xy.x / P->k0;
    return lp;
}


// static LP s_inverse (XY xy, PJ *P) {           /* Spheroidal, inverse */
//     LP lp = {0.0,0.0};
//     lp.phi = M_HALFPI - 2. * atan(exp(-xy.y / P->k0));
//     lp.lam = xy.x / P->k0;
//     return lp;
// }


PJ *PROJECTION(merc_nav) {
    double phits=0.0;
    int is_phits;

    if( (is_phits = pj_param(P->ctx, P->params, "tlat_ts").i) ) {
        phits = fabs(pj_param(P->ctx, P->params, "rlat_ts").f);
        if (phits >= M_HALFPI)
            return pj_default_destructor(P, PJD_ERR_LAT_TS_LARGER_THAN_90);
    }

    if (P->es != 0.0) { /* ellipsoid */
        if (is_phits)
            P->k0 = pj_msfn(sin(phits), cos(phits), P->es);
        P->inv = e_inverse;
        P->fwd = e_forward;
    }

    // else { /* sphere */
    //     if (is_phits)
    //         P->k0 = cos(phits);
    //     P->inv = s_inverse;
    //     P->fwd = s_forward;
    // }

    return P;
}


// #ifdef PJ_OMIT_SELFTEST
// int pj_nav_merc_selftest (void) {return 0;}
// #else
// 
// int pj_nav_merc_selftest (void) {
    // double tolerance_lp = 1e-10;
    // double tolerance_xy = 1e-7;
// 
    // char e_args[] = {" -w6 +proj=nav_merc +ellps=intl"};
// 
    // LP fwd_in[] = {
        // { 950594.539, 5968306.230 },
        // { 1178796.03, 5276463.78 },
        // { 18671158.340, -5945578.371 },
        // { 20038296.88, -18765191.35 }
    // };
// 
    // XY e_fwd_expect[] = {
        // { 8.5390000000, 47.3515666667 },
        // { 10.5888881683, 42.9574966432 },
        // { 167.7192687988, -47.2126350403 },
        // { 179.9998888889, -84.0 },
    // };
// 
    // XY inv_in[] = {
        // { 8.5390000000, 47.3515666667 },
        // { 10.5888881683, 42.9574966432 },
        // { 167.7192687988, -47.2126350403 },
        // { 180.0, -84.0 }
    // };
// 
    // LP e_inv_expect[] = {
        // { 950594.539, 5968306.230 },
        // { 1178796.03, 5276463.78 },
        // { 18671158.340, -5945578.371 },
        // { 20038296.883, -18765191.347 },
    // };
// 
// 
    // return pj_generic_selftest (e_args, 0, tolerance_xy, tolerance_lp, 4, 4, fwd_in, 0, e_fwd_expect, inv_in, 0, e_inv_expect );
// }
// 
// 
// #endif
