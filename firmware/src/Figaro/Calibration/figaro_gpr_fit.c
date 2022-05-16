/**
 * @file    figaro_gpr_fit.c
 *
 * @brief   Functions to convert figaro impedance to
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

#include "mpfit.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846264338327950288
#endif

#ifndef NAN
#define NAN (0.0/0.0)
#endif
#define UNUSED(x) (void)(x)

#include "figaro_gpr_fit.h"
#include "figaro_gpr_params.h"
// double curveData[6];
double dotProduct(const double v[], const double u[], const int n);
double expKernel(double x, const double sigmaF, const double sigmal);
int figaro_mox_func(int m, int n, double *p, double *dy, double **dvec, void *vars);
int figaro_hill_func(int m, int n, double *p, double *dy, double **dvec, void *vars);

#define NUM_GPR_6PARAM_COEFFS 360
#define NUM_GPR_R_AH_T_PARAM_COEFFS 616

typedef struct impedance_fit_func_vars_struct
{
	double *x;
	double *y;
	double *ey;
	bool *is_real;
} impedance_fit_func_vars_t;

int figaro_hill_func(int m, int n, double *p, double *dy, double **dvec, void *vars)
{
	UNUSED(dvec);
	UNUSED(n);
	impedance_fit_func_vars_t *v = (impedance_fit_func_vars_t *)vars;
	double *x, *y, *ey, f, t;

	x = v->x;
	y = v->y;
	ey = v->ey;

	for (int i = 0; i < m; i++)
	{
		t = (log(x[i] / p[2])) / p[3];
		f = p[0] + p[1] * exp(-1 * pow(t, 2));
		dy[i] = (y[i] - f) / ey[i];
	}

	return 0;
}

int figaro_mox_func(int m, int n, double *p, double *dy, double **dvec, void *vars)
{
	UNUSED(dvec);
	UNUSED(n);
	if (n != 2)
		return -1;
	impedance_fit_func_vars_t *v = (impedance_fit_func_vars_t *)vars;
	double *x, *y, *ey, f;
	bool *is_real;
	double R = p[0];
	double C = p[1];

	x = v->x;
	y = v->y;
	ey = v->ey;
	is_real = v->is_real;

	for (int i = 0; i < m; i++)
	{
		double rc2 = R * C * 2 * M_PI * x[i];
		if (is_real[i])
		{
			f = R / (1 + rc2 * rc2);
		}
		else
		{
			f = (-R * rc2) / (1 + rc2 * rc2);
		}
		dy[i] = (y[i] - f) / ey[i];
	}
	return 0;
}

FIGAROFIT_EXPORT figaro_result_t figaro_imaginary_hill_func_fit_default(double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double *p)
{
	double x[DEFAULT_NUM_IMPEDANCE_POINTS] = {2000, 8166.67, 14333.33, 20500, 26666.67, 32833.33, 39000, 45166.67,
											  51333.33, 57500, 63666.67, 69833.33, 76000, 82166.67, 88333.33, 94500, 100666.67,
											  106833.33, 113000, 119166.67, 125333.33, 131500, 137666.67, 143833.33, 150000};
	double ey[DEFAULT_NUM_IMPEDANCE_POINTS];
	for (int i = 0; i < DEFAULT_NUM_IMPEDANCE_POINTS; i++)
		ey[i] = 1.0;
	return figaro_imaginary_hill_func_fit(x, imag, ey, DEFAULT_NUM_IMPEDANCE_POINTS, p, 4);
}

FIGAROFIT_EXPORT figaro_result_t figaro_imaginary_hill_func_fit(double *x, double *y, double *ey, size_t num_pts, double *p, size_t num_params)
{
	// double p[] = { -61, -7895, 46914, 0.5737 };        /* Initial conditions */
	if (num_params != 4)
		return FIGARO_ERR_BAD_PARAMS;
	p[0] = -61.0;
	p[1] = -7895;
	p[2] = 46914;
	p[3] = 0.5737;
	// double p[] = { -61, -7895, 46914, 1.38 };        /* Initial conditions */
	double perror[4]; /* Returned parameter errors */
	mp_par pars[4];	  /* Parameter constraints */
	impedance_fit_func_vars_t v;
	int status;
    UNUSED(status);
	mp_result result;

	memset(&result, 0, sizeof(result)); /* Zero results structure */
	result.xerror = perror;
	memset(pars, 0, sizeof(pars)); /* Initialize constraint structure */

	pars[0].limited[0] = 1;
	pars[0].limited[1] = 1;
	pars[0].limits[0] = -1062;
	pars[0].limits[1] = -60;

	pars[1].limited[0] = 1;
	pars[1].limited[1] = 1;
	pars[1].limits[0] = -12290;
	pars[1].limits[1] = -270;

	pars[2].limited[0] = 1;
	pars[2].limited[1] = 1;
	pars[2].limits[0] = 4251.3;
	pars[2].limits[1] = 89577;

	pars[3].limited[0] = 1;
	pars[3].limited[1] = 1;
	// pars[3].limits[0] = 1.38;
	pars[3].limits[0] = 0;
	pars[3].limits[1] = 1.579;

	v.x = x;
	v.y = y;
	v.ey = ey;

	/* Call fitting function for 25 data points and 4 parameters */
	status = mpfit(figaro_hill_func, (int)num_pts, (int)num_params, p, pars, 0, (void *)&v, &result);

	return FIGARO_OK;
}

FIGAROFIT_EXPORT figaro_result_t figaro_r_mox_func_fit_default(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double *p)
{
	double x[2 * DEFAULT_NUM_IMPEDANCE_POINTS] = {2000, 8166.67, 14333.33, 20500, 26666.67, 32833.33, 39000, 45166.67,
												  51333.33, 57500, 63666.67, 69833.33, 76000, 82166.67, 88333.33, 94500, 100666.67,
												  106833.33, 113000, 119166.67, 125333.33, 131500, 137666.67, 143833.33, 150000,
												  2000, 8166.67, 14333.33, 20500, 26666.67, 32833.33, 39000, 45166.67,
												  51333.33, 57500, 63666.67, 69833.33, 76000, 82166.67, 88333.33, 94500, 100666.67,
												  106833.33, 113000, 119166.67, 125333.33, 131500, 137666.67, 143833.33, 150000};
	double ey[2 * DEFAULT_NUM_IMPEDANCE_POINTS];
	bool is_real[2 * DEFAULT_NUM_IMPEDANCE_POINTS];
	double y[2 * DEFAULT_NUM_IMPEDANCE_POINTS];
	for (int i = 0; i < 2 * DEFAULT_NUM_IMPEDANCE_POINTS; i++)
	{
		ey[i] = 1.0;
		if (i < DEFAULT_NUM_IMPEDANCE_POINTS)
		{
			is_real[i] = true;
			y[i] = real[i];
		}
		else
		{
			is_real[i] = false;
			y[i] = imag[i - DEFAULT_NUM_IMPEDANCE_POINTS];
		}
	}
	return figaro_r_mox_func_fit(x, y, is_real, ey, 2 * DEFAULT_NUM_IMPEDANCE_POINTS, p, 2);
}

FIGAROFIT_EXPORT figaro_result_t figaro_r_mox_func_fit(double *x, double *y, bool *is_real, double *ey, size_t num_pts, double *p, size_t num_params)
{
	// double p[] = { -61, -7895, 46914, 0.5737 };        /* Initial conditions */
	if (num_params != 2)
		return FIGARO_ERR_BAD_PARAMS;
	p[1] = 1025.8e-12;
	double maxR = 0.0;
	for (size_t i = 0; i < num_pts; i++)
	{
		if ((is_real[i]) && y[i] > maxR)
			maxR = y[i];
	}
	p[0] = maxR;
	double perror[2]; /* Returned parameter errors */
	mp_par pars[2];	  /* Parameter constraints */
	impedance_fit_func_vars_t v;
	int status;
	mp_result result;

	memset(&result, 0, sizeof(result)); /* Zero results structure */
	result.xerror = perror;
	memset(pars, 0, sizeof(pars)); /* Initialize constraint structure */

	pars[0].limited[0] = 1;
	pars[0].limited[1] = 1;
	pars[0].limits[0] = 1e-3;
	pars[0].limits[1] = 100000;
	if (p[0]<pars[0].limits[0]) {p[0]=pars[0].limits[0]+1e-3;}
	if (p[0]>pars[0].limits[1]) {p[0]=pars[0].limits[1]-1e-3;}
	pars[1].fixed = true;

	v.x = x;
	v.y = y;
	v.ey = ey;
	v.is_real = is_real;

	/* Call fitting function for 25 data points and 4 parameters */
	status = mpfit(figaro_mox_func, (int)num_pts, (int)num_params, p, pars, 0, (void *)&v, &result);
	double sumy= 0.0;
	for (size_t i = 0; i<num_pts; i++) sumy+=y[i]*y[i];
	double normalized_chisqr = result.bestnorm/sumy;
	if (!((status > 0) && (status <= 4)))
	{
		switch (status)
		{
			case MP_MAXITER:
				return FIGARO_ERR_MP_MAXITER;
				break;
			case MP_FTOL:
				return FIGARO_ERR_MP_FTOL;
				break;
			case MP_XTOL:
				return FIGARO_ERR_MP_XTOL;
				break;
			case MP_GTOL:
				return FIGARO_ERR_MP_GTOL;
				break;
			case MP_ERR_NAN:
			case MP_ERR_FUNC:
			case MP_ERR_NPOINTS:
			case MP_ERR_NFREE:
			case MP_ERR_MEMORY:
			case MP_ERR_INITBOUNDS:
			case MP_ERR_BOUNDS:
			case MP_ERR_PARAM:
			case MP_ERR_DOF:
				return FIGARO_ERR_MP_INTERNAL;
				break;
			default:
				return FIGARO_ERR_FIT_NOT_CONVERGED;
		}		
	}
	if (normalized_chisqr>5e-2)
	{
		return FIGARO_ERR_BAD_FIT_CHI_SQ;
	}
	if (p[0]<100.0)
	{
		return FIGARO_ERR_LOW_RESISTANCE;
	}
	return FIGARO_OK;
}

double expKernel(double x, const double sigmaF, const double sigmal)
{
	double retVal;
	retVal = pow(sigmaF, 2) * exp(-1 * x / sigmal);
	return retVal;
}

double dotProduct(const double v[], const double u[], const int n)
{
	double result = 0;
	for (int i = 0; i < n; i++)
	{
		result += v[i] * u[i];
	}
	return result;
}

FIGAROFIT_EXPORT double predFigaroGPRImag6Param(double hill_fit[4], double T, double RH)
{
	double value[6];
	value[0] = hill_fit[0];
	value[1] = hill_fit[1];
	value[2] = hill_fit[2];
	value[3] = hill_fit[3];
	value[4] = T;
	value[5] = RH;
	return predFigaroGPR(value,
						 FigaroGPRImag_6Param.num_params,
						 FigaroGPRImag_6Param.num_coeffs,
						 FigaroGPRImag_6Param.xOffset,
						 FigaroGPRImag_6Param.scaleX,
						 (double *)(FigaroGPRImag_6Param.xTrained),
						 FigaroGPRImag_6Param.alpha,
						 FigaroGPRImag_6Param.sigmaF,
						 FigaroGPRImag_6Param.sigmal,
						 FigaroGPRImag_6Param.yOffset);
}

FIGAROFIT_EXPORT double predFigaroGPRR3Param(double R, double T, double RH)
{
	double value[3];
	value[0] = R;
	value[1] = T;
	value[2] = RH;
	return predFigaroGPR(value,
						 FigaroGPRR_3Param.num_params,
						 FigaroGPRR_3Param.num_coeffs,
						 FigaroGPRR_3Param.xOffset,
						 FigaroGPRR_3Param.scaleX,
						 (double *)(FigaroGPRR_3Param.xTrained),
						 FigaroGPRR_3Param.alpha,
						 FigaroGPRR_3Param.sigmaF,
						 FigaroGPRR_3Param.sigmal,
						 FigaroGPRR_3Param.yOffset);
}

FIGAROFIT_EXPORT double predFigaroGPRR2Param(double R, double AH)
{
	double value[2];
	value[0] = R;
	value[1] = AH;
	return predFigaroGPR(value,
						 FigaroGPRR_2Param.num_params,
						 FigaroGPRR_2Param.num_coeffs,
						 FigaroGPRR_2Param.xOffset,
						 FigaroGPRR_2Param.scaleX,
						 (double *)(FigaroGPRR_2Param.xTrained),
						 FigaroGPRR_2Param.alpha,
						 FigaroGPRR_2Param.sigmaF,
						 FigaroGPRR_2Param.sigmal,
						 FigaroGPRR_2Param.yOffset);
}

FIGAROFIT_EXPORT double predFigaroGPR(double *value,
									  const size_t num_params,
									  const size_t num_coeffs,
									  const double *xOffset,
									  const double *scaleX,
									  const double *xTrained,
									  const double *alpha,
									  const double sigmaF,
									  const double sigmal,
									  const double yOffset)
{
	for (size_t i = 0; i < num_params; i++)
	{
		value[i] = (value[i] - xOffset[i]) / scaleX[i];
	}
	double retVal = yOffset;
	for (size_t j = 0; j < num_coeffs; j++)
	{
		double sum = 0;
		for (size_t i = 0; i < num_params; i++)
		{
			double distTemp;
			distTemp = (value[i] - xTrained[j * num_params + i]);
			sum += distTemp * distTemp;
		}
		retVal += alpha[j] * expKernel(sqrt(sum), sigmaF, sigmal);
	}
	return retVal;
}

FIGAROFIT_EXPORT figaro_result_t figaro_r_t_rh_calibration_realimag_error(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C)
{
	double p[2];
	figaro_result_t res = figaro_r_mox_func_fit_default(real, imag, p);
	if (res != FIGARO_OK)
	{
		*ppm = NAN;
		*R = NAN;
		*C = NAN;
		return res;
	}
	*ppm = predFigaroGPRR3Param(p[0], T, RH);
	*R = p[0];
	*C = p[1];
	return FIGARO_OK;
}

FIGAROFIT_EXPORT double figaro_r_t_rh_calibration_realimag(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH)
{
	double ppm, R, C;
	figaro_result_t res = figaro_r_t_rh_calibration_realimag_error(real, imag, T, RH, &ppm, &R, &C);
	if (res == FIGARO_OK)
	{
		return ppm;
	} 
	else
	{
		return NAN;
	}
}

FIGAROFIT_EXPORT figaro_result_t figaro_r_t_rh_calibration_error(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C)
{
	double p[2];
	double real[DEFAULT_NUM_IMPEDANCE_POINTS], imag[DEFAULT_NUM_IMPEDANCE_POINTS];
	figaro_result_t res = convert_polar_to_realimag(magnitude, angle_deg, real, imag, DEFAULT_NUM_IMPEDANCE_POINTS);
	if (res != FIGARO_OK)
	{
		*ppm = NAN;
		*R = NAN;
		*C = NAN;
		return res;
	}	
	res = figaro_r_mox_func_fit_default(real, imag, p);
	if (res != FIGARO_OK)
	{
		*ppm = NAN;
		*R = NAN;
		*C = NAN;
		return res;
	}	
	*ppm = predFigaroGPRR3Param(p[0], T, RH);
	*R = p[0];
	*C = p[1];
	return FIGARO_OK;
}

FIGAROFIT_EXPORT double figaro_r_t_rh_calibration(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH)
{
	double ppm, R, C;
	figaro_result_t res = figaro_r_t_rh_calibration_error(magnitude, angle_deg, T, RH, &ppm, &R, &C);
	if (res == FIGARO_OK)
	{
		return ppm;
	} 
	else
	{
		return NAN;
	}
}
FIGAROFIT_EXPORT figaro_result_t convert_polar_to_realimag(double *magnitude, double *angle_deg, double *real, double *imag, size_t npoints)
{
	for (size_t i = 0; i < npoints; i++)
	{
		double a = angle_deg[i] * M_PI / 180.0;
		double m = magnitude[i];
		real[i] = m * cos(a);
		imag[i] = m * sin(a);
	}
	return FIGARO_OK;
}

FIGAROFIT_EXPORT double convert_T_RH_to_AH(double T, double RH)
{
	return (6.112 * exp((17.67 * T)/(T+243.5)) * RH * 2.1674)/ (273.15+T);
}

FIGAROFIT_EXPORT double figaro_r_ah_calibration_realimag(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH)
{
	double ppm, R, C;
	figaro_result_t res = figaro_r_ah_calibration_realimag_error(real, imag, T, RH, &ppm, &R, &C);
	if (res == FIGARO_OK)
	{
		return ppm;
	} 
	else
	{
		return NAN;
	}
}

FIGAROFIT_EXPORT figaro_result_t figaro_r_ah_calibration_realimag_error(double real[DEFAULT_NUM_IMPEDANCE_POINTS], double imag[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C)
{
	double p[2];
	figaro_result_t res = figaro_r_mox_func_fit_default(real, imag, p);
	if (res != FIGARO_OK)
	{
		*ppm = NAN;
		*R = NAN;
		*C = NAN;
		return res;
	}
	double AH = convert_T_RH_to_AH(T, RH);
	*ppm = predFigaroGPRR2Param(p[0], AH);
	*R = p[0];
	*C = p[1];
	return FIGARO_OK;
}

FIGAROFIT_EXPORT double figaro_r_ah_calibration(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH)
{
	double ppm, R, C;
	figaro_result_t res = figaro_r_ah_calibration_error(magnitude, angle_deg, T, RH, &ppm, &R, &C);
	if (res == FIGARO_OK)
	{
		return ppm;
	} 
	else
	{
		return NAN;
	}
}

FIGAROFIT_EXPORT figaro_result_t figaro_r_ah_calibration_error(double magnitude[DEFAULT_NUM_IMPEDANCE_POINTS], double angle_deg[DEFAULT_NUM_IMPEDANCE_POINTS], double T, double RH, double* ppm, double* R, double* C)
{
	double p[2];
	double real[DEFAULT_NUM_IMPEDANCE_POINTS], imag[DEFAULT_NUM_IMPEDANCE_POINTS];
	figaro_result_t res = convert_polar_to_realimag(magnitude, angle_deg, real, imag, DEFAULT_NUM_IMPEDANCE_POINTS);
	if (res != FIGARO_OK)
	{
		*ppm = NAN;
		*R = NAN;
		*C = NAN;		
		return res;
	}	
	res = figaro_r_mox_func_fit_default(real, imag, p);
	if (res != FIGARO_OK)
	{
		*ppm = NAN;
		*R = NAN;
		*C = NAN;		
		return res;
	}	
	double AH = convert_T_RH_to_AH(T, RH);
	*ppm = predFigaroGPRR2Param(p[0], AH);
	*R = p[0];
	*C = p[1];
	return FIGARO_OK;
}
