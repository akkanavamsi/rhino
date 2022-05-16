/**
 * @file    figaro_gpr_params.h
 *
 * @brief   Definition of GPR calibration model structs
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
extern "C" {
#endif

#include <stddef.h>
//number of input parameters to GPR
#define FigaroGPRImag_6Param_num_params 6
//number of fit points (e.g. coefficients) in GPR model
#define FigaroGPRImag_6Param_num_coeffs 360

typedef struct FigaroGPRImag_6Param_struct {
    size_t num_params;
    size_t num_coeffs;
    double xOffset[FigaroGPRImag_6Param_num_params];
    double scaleX[FigaroGPRImag_6Param_num_params];
    double xTrained[FigaroGPRImag_6Param_num_coeffs][FigaroGPRImag_6Param_num_params];
    double alpha[FigaroGPRImag_6Param_num_coeffs];
	double sigmaF;
	double sigmal;
	double yOffset;
} FigaroGPRImag_6Param_struct_t;

//this is a 6 parameter fit for imaginary 'y0', 'A', 'x0', 'width' plus RH and T
extern FigaroGPRImag_6Param_struct_t FigaroGPRImag_6Param;

//number of input parameters to GPR
#define FigaroGPRR_3Param_num_params 3
//number of fit points (e.g. coefficients) in GPR model
#define FigaroGPRR_3Param_num_coeffs 1232
typedef struct FigaroGPRR_3Param_struct {
    size_t num_params;
    size_t num_coeffs;
    double xOffset[FigaroGPRR_3Param_num_params];
    double scaleX[FigaroGPRR_3Param_num_params];
    double xTrained[FigaroGPRR_3Param_num_coeffs][FigaroGPRR_3Param_num_params];
    double alpha[FigaroGPRR_3Param_num_coeffs];
	double sigmaF;
	double sigmal;
	double yOffset;
} FigaroGPRR_3Param_struct_t;

//this is a 3 parameter fit for fitted R plus relative humidity and temperature
extern FigaroGPRR_3Param_struct_t FigaroGPRR_3Param;

//number of input parameters to GPR
#define FigaroGPRR_2Param_num_params 2
//number of fit points (e.g. coefficients) in GPR model
#define FigaroGPRR_2Param_num_coeffs 600
typedef struct FigaroGPRR_2Param_struct {
    size_t num_params;
    size_t num_coeffs;
    double xOffset[FigaroGPRR_2Param_num_params];
    double scaleX[FigaroGPRR_2Param_num_params];
    double xTrained[FigaroGPRR_2Param_num_coeffs][FigaroGPRR_2Param_num_params];
    double alpha[FigaroGPRR_2Param_num_coeffs];
	double sigmaF;
	double sigmal;
	double yOffset;
} FigaroGPRR_2Param_struct_t;

//this is a 2 parameter fit for fitted R plus absolute humidity
extern FigaroGPRR_2Param_struct_t FigaroGPRR_2Param;


#ifdef __cplusplus
}
#endif