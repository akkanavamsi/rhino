/**
 * @file    figaro_gpr_fit.h
 *
 * @brief   Exported functions to convert figaro impedance to
 *          methane concentration
 *
 * @attention
 * (c) Copyright Schlumberger Technology Corp., unpublished work,
 * created 2022.  This computer program includes confidential,
 * proprietary information and is a trade secret of Schlumberger
 * Technology Corp.  All use, disclosure, and/or reproduction is
 * prohibited unless authorized in writing.  All Rights Reserved.
 */
// *******************************************************************************

#pragma once
#ifdef __cplusplus
extern "C"
{
#endif
    #include <stddef.h>
    #include <stdbool.h>
    #include "figarofit_export.h"

    #define DEFAULT_NUM_IMPEDANCE_POINTS 25

    typedef enum 
    { 
        FIGARO_OK, 
        FIGARO_ERR_BAD_PARAMS,
        FIGARO_ERR_FIT_NOT_CONVERGED, 
        FIGARO_ERR_BAD_FIT_CHI_SQ,
        FIGARO_ERR_LOW_RESISTANCE,
        FIGARO_ERR_MP_MAXITER,
        FIGARO_ERR_MP_FTOL,
        FIGARO_ERR_MP_XTOL,
        FIGARO_ERR_MP_GTOL,
        FIGARO_ERR_MP_INTERNAL
    } figaro_result_t;

    // fits array of x,y, and error pts of size num_pts to the hill function.  Expects p to be of size 4 (and specified in num_params)
    FIGAROFIT_EXPORT figaro_result_t figaro_imaginary_hill_func_fit(double *x, double *y, double *ey, size_t num_pts, double *p, size_t num_params);

    // fits array of x,y, is_real, and error pts of size num_pts to the parallel RC impedance function.  Note is_real defines whether a given point is real or imaginary.
    // Return: p of size 2 (and specified in num_params)
    FIGAROFIT_EXPORT figaro_result_t figaro_r_mox_func_fit(double *x, double *y, bool *is_real, double *ey, size_t num_pts, double *p, size_t num_params);

    // converts polar array of magnitude and angle with size npoints to cartesian real and imaginary.  Note to save memory the input and output arrays can be the same.
    // In this case the input is overwritten.
    FIGAROFIT_EXPORT figaro_result_t convert_polar_to_realimag(double* magnitude, double* angle_deg, double* real, double* imag, size_t npoints);

    // converts temperature and relative humidty to absolute humidity using Clausius equation
    FIGAROFIT_EXPORT double convert_T_RH_to_AH(double T, double RH);

    // generic function to run GPR model defined by coefficients given in xOffset ... yOffset
    // input is value array of size num_params
    // output is predicted value
    FIGAROFIT_EXPORT double predFigaroGPR(double *value,
                                          const size_t num_params,
                                          const size_t num_coeffs,
                                          const double *xOffset, //num_params
                                          const double *scaleX, //num_params
                                          const double *xTrained, //row major (num_coeffs x num_params)
                                          const double *alpha, //num_coeffs
                                          const double sigmaF,
                                          const double sigmal,
                                          const double yOffset);

    // runs 6 parameter GPR using outputs of hill function fit, T, and RH
    FIGAROFIT_EXPORT double predFigaroGPRImag6Param(double hill_fit[4], double T, double RH);

    // runs 3 parameter GPR using fitted R value from mox_func_fit, temperature, and relative humidity
    FIGAROFIT_EXPORT double predFigaroGPRR3Param(double R, double T, double RH);

    // runs 2 parameter GPR using fitted R value from mox_func_fit and absolute humidity
    FIGAROFIT_EXPORT double predFigaroGPRR2Param(double R, double AH);

    // simplified hill function fit that uses default x values for frequencies
    FIGAROFIT_EXPORT figaro_result_t figaro_imaginary_hill_func_fit_default(double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double *p);

    // simplified parallel RC impedance function fit that uses default x values for frequencies
    FIGAROFIT_EXPORT figaro_result_t figaro_r_mox_func_fit_default(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double *p);

    // simplified function to fit R to real and imaginary points (with default frequencies) and then run 3 parameter GPR fit.  Returns predicted concentration or NaN if an error
    FIGAROFIT_EXPORT double figaro_r_t_rh_calibration_realimag(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH);

    // simplified function to fit R to magnitude and phase points (with default frequencies) and then run 3 parameter GPR fit.  Returns predicted concentration or NaN if an error
    FIGAROFIT_EXPORT double figaro_r_t_rh_calibration(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH);

    // simplified function to fit R to real and imaginary points (with default frequencies) and then run 2 parameter GPR fit.  Returns predicted concentration or NaN if an error
    FIGAROFIT_EXPORT double figaro_r_ah_calibration_realimag(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH);

    // simplified function to fit R to magnitude and phase points (with default frequencies) and then run 2 parameter GPR fit.  Returns predicted concentration or NaN if an error
    FIGAROFIT_EXPORT double figaro_r_ah_calibration(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH);

    // simplified function to fit R to real and imaginary points (with default frequencies) and then run 3 parameter GPR fit.  
    // inputs:
    //         real = array of real impedances (ohms)
    //         imag = array of imaginary impedances (ohms)
    //         T = temperature (degC)
    //         RH = relative humidity (%)
    // outputs:
    //         *ppm = concentration value or NAN if error
    //         *R = fitted R value or NAN if error
    //         *C = fitted C value or NAN if error
    //         return value: Returns status.
    FIGAROFIT_EXPORT figaro_result_t figaro_r_t_rh_calibration_realimag_error(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C);

    // simplified function to fit R to magnitude and phase points (with default frequencies) and then run 3 parameter GPR fit.  
    // inputs:
    //         magnitude = array of impedance magnitudes (ohms)
    //         angle_deg = array of impedance phases (degrees)
    //         T = temperature (degC)
    //         RH = relative humidity (%)
    // outputs:
    //         *ppm = concentration value or NAN if error
    //         *R = fitted R value or NAN if error
    //         *C = fitted C value or NAN if error
    //         return value: Returns status.
    FIGAROFIT_EXPORT figaro_result_t figaro_r_t_rh_calibration_error(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C);

    // simplified function to fit R to real and imaginary points (with default frequencies) and then run 2 parameter GPR fit.  
    // inputs:
    //         real = array of real impedances (ohms)
    //         imag = array of imaginary impedances (ohms)
    //         T = temperature (degC)
    //         RH = relative humidity (%)
    // outputs:
    //         *ppm = concentration value or NAN if error
    //         *R = fitted R value or NAN if error
    //         *C = fitted C value or NAN if error
    //         return value: Returns status.
    FIGAROFIT_EXPORT figaro_result_t figaro_r_ah_calibration_realimag_error(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C);

    // simplified function to fit R to magnitude and phase points (with default frequencies) and then run 3 parameter GPR fit.  
    // inputs:
    //         magnitude = array of impedance magnitudes (ohms)
    //         angle_deg = array of impedance phases (degrees)
    //         T = temperature (degC)
    //         RH = relative humidity (%)
    // outputs:
    //         *ppm = concentration value or NAN if error
    //         *R = fitted R value or NAN if error
    //         *C = fitted C value or NAN if error
    //         return value: Returns status.
    FIGAROFIT_EXPORT figaro_result_t figaro_r_ah_calibration_error(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C);

#ifdef __cplusplus
}
#endif