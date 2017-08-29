/*
 * Sponsored License - for use in support of a program or activity
 * sponsored by MathWorks.  Not for government, commercial or other
 * non-sponsored organizational use.
 * File: predictFault.c
 *
 * MATLAB Coder version            : 3.3
 * C/C++ source code generated on  : 28-Aug-2017 11:27:58
 */

/* Include Files */
#include "predictFault.h"

/* Function Declarations */
static void Linear(const float svT[54], const float x[6], float kernelProduct[9]);
static void c_CompactClassificationSVM_norm(const double obj_Mu[6], const double
  obj_Sigma[6], float X[6]);

/* Function Definitions */

/*
 * Arguments    : const float svT[54]
 *                const float x[6]
 *                float kernelProduct[9]
 * Return Type  : void
 */
static void Linear(const float svT[54], const float x[6], float kernelProduct[9])
{
  int i0;
  int i1;
  for (i0 = 0; i0 < 9; i0++) {
    kernelProduct[i0] = 0.0F;
    for (i1 = 0; i1 < 6; i1++) {
      kernelProduct[i0] += x[i1] * svT[i1 + 6 * i0];
    }
  }
}

/*
 * Arguments    : const double obj_Mu[6]
 *                const double obj_Sigma[6]
 *                float X[6]
 * Return Type  : void
 */
static void c_CompactClassificationSVM_norm(const double obj_Mu[6], const double
  obj_Sigma[6], float X[6])
{
  int partialTrueCount;
  int ak;
  float a[6];
  int bk;
  int trueCount;
  int ck;
  int asub;
  int tmp_data[6];
  int bsub;
  int loop_ub;
  int b_tmp_data[6];
  signed char c_tmp_data[6];
  for (partialTrueCount = 0; partialTrueCount < 6; partialTrueCount++) {
    a[partialTrueCount] = X[partialTrueCount];
  }

  ak = 0;
  bk = 0;
  trueCount = 0;
  for (ck = 0; ck < 6; ck++) {
    X[ck] = a[ak] - (float)obj_Mu[bk];
    ak++;
    bk++;
    if (obj_Sigma[ck] > 0.0) {
      trueCount++;
    }
  }

  partialTrueCount = 0;
  for (ak = 0; ak < 6; ak++) {
    if (obj_Sigma[ak] > 0.0) {
      tmp_data[partialTrueCount] = ak + 1;
      partialTrueCount++;
    }
  }

  if ((signed char)trueCount != 0) {
    asub = 1;
    bsub = 1;
    ak = 0;
    bk = 0;
    if (0 <= (signed char)trueCount - 1) {
      loop_ub = trueCount;
    }

    for (ck = 0; ck < (signed char)trueCount; ck++) {
      for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++)
      {
        c_tmp_data[partialTrueCount] = (signed char)tmp_data[partialTrueCount];
      }

      a[ck] = X[c_tmp_data[ak] - 1] / (float)obj_Sigma[tmp_data[bk] - 1];
      if (asub < loop_ub) {
        ak++;
        bk++;
        bsub++;
        asub++;
      } else if (bsub < trueCount) {
        bk++;
        bsub++;
      } else {
        asub = 1;
        bsub = 1;
      }
    }
  }

  partialTrueCount = 0;
  for (ak = 0; ak < 6; ak++) {
    if (obj_Sigma[ak] > 0.0) {
      b_tmp_data[partialTrueCount] = ak + 1;
      partialTrueCount++;
    }
  }

  loop_ub = (signed char)trueCount;
  for (partialTrueCount = 0; partialTrueCount < loop_ub; partialTrueCount++) {
    X[b_tmp_data[partialTrueCount] - 1] = a[partialTrueCount];
  }
}

/*
 * PREDICFAULT Classify if the flight is in fault or not based on compact
 *    model classificationSVMFault
 *    to generate this compact model saved in mat file. run the row below first
 *    saveCompactModel(SVMModel,'classificationSVMFault');
 *    PREDICTFAULT classifies the 28-by-28 images in the rows of X using
 *    the compact ECOC model in the file classificationSVMFault.mat, and then
 *    returns class labels in label.
 * Arguments    : const float X[6]
 *                cell_wrap_0 label[1]
 * Return Type  : void
 */
void predictFault(const float X[6], cell_wrap_0 label[1])
{
  double CompactMdl_Alpha[9];
  static const double dv0[9] = { 0.93637209437248092, 0.10080279622684034,
    0.21716853050107243, 0.36051657918116736, 0.99999999999999989,
    0.99999999999999989, 1.0, 0.08186188613429661, 0.53299811414726506 };

  int i;
  double CompactMdl_Mu[6];
  static const double dv1[6] = { -1.6450072306244061, -0.18322448383547493,
    -25.069354269773918, -33.2402215198353, 55.097029858434183,
    -25.934079877201796 };

  double CompactMdl_Sigma[6];
  static const double dv2[6] = { 0.84752699043434643, 0.1004482461690703,
    5.67146688916856, 15.336435651029708, 13.941239749143028, 2.1454090507324195
  };

  signed char CompactMdl_SupportVectorLabels[9];
  static const signed char iv0[9] = { -1, -1, -1, -1, 1, 1, -1, 1, 1 };

  char CompactMdl_ClassNames[14];
  static const char cv0[14] = { 'n', 'f', 'o', 'a', 'm', 'u', 'i', 'l', 'n', 't',
    'a', ' ', 'l', ' ' };

  char CompactMdl_NonzeroProbClasses[14];
  float z[6];
  static const float fv0[54] = { -0.676036477F, 1.81275403F, -0.661854267F,
    -0.65948993F, 0.755818605F, -0.173748031F, 2.0057466F, 1.1877681F,
    1.78806007F, 0.073705256F, -1.36448085F, -4.3389473F, 5.2728076F,
    -0.661568403F, 3.08425927F, 1.79698122F, -4.31694651F, 10.8353329F,
    -0.686127603F, -0.106546983F, -0.655544519F, -0.651952565F, 0.717508793F,
    -0.179099143F, -0.836014926F, 0.698847592F, -0.825192928F, -0.839062274F,
    0.628306389F, 0.61070776F, -0.783307612F, 1.72078681F, -0.829392F,
    0.346366197F, 0.60637933F, 1.58466315F, -0.661688209F, -0.577857435F,
    -0.59934324F, -0.633782685F, 0.738590598F, -0.402120888F, 1.49886692F,
    -1.51892388F, 1.50642371F, 1.40249932F, -1.28638685F, -1.65372479F,
    1.46562803F, -2.30047536F, 1.49900043F, 1.40548015F, -1.23892987F,
    -1.5879178F };

  float fv1[9];
  float f0;
  int k;
  double alphas;
  float scores[2];
  double b_alphas[9];
  int iloc[2];
  boolean_T exitg1;
  boolean_T rowmatch;
  int loop_ub;
  boolean_T exitg2;
  memcpy(&CompactMdl_Alpha[0], &dv0[0], 9U * sizeof(double));
  for (i = 0; i < 6; i++) {
    CompactMdl_Mu[i] = dv1[i];
    CompactMdl_Sigma[i] = dv2[i];
  }

  for (i = 0; i < 9; i++) {
    CompactMdl_SupportVectorLabels[i] = iv0[i];
  }

  for (i = 0; i < 14; i++) {
    CompactMdl_ClassNames[i] = cv0[i];
    CompactMdl_NonzeroProbClasses[i] = cv0[i];
  }

  for (i = 0; i < 6; i++) {
    z[i] = X[i];
  }

  c_CompactClassificationSVM_norm(CompactMdl_Mu, CompactMdl_Sigma, z);
  Linear(fv0, z, fv1);
  f0 = 0.0F;
  for (i = 0; i < 9; i++) {
    alphas = CompactMdl_Alpha[i] * (double)CompactMdl_SupportVectorLabels[i];
    f0 += fv1[i] * (float)alphas;
    b_alphas[i] = alphas;
  }

  for (k = 0; k < 2; k++) {
    scores[k] = -(f0 + -0.385483027F);
    iloc[k] = 0;
    i = 1;
    exitg1 = false;
    while ((!exitg1) && (i < 3)) {
      rowmatch = true;
      loop_ub = 0;
      exitg2 = false;
      while ((!exitg2) && (loop_ub + 1 < 8)) {
        if (!(CompactMdl_NonzeroProbClasses[k + (loop_ub << 1)] ==
              CompactMdl_ClassNames[(i + (loop_ub << 1)) - 1])) {
          rowmatch = false;
          exitg2 = true;
        } else {
          loop_ub++;
        }
      }

      if (rowmatch) {
        iloc[k] = i;
        exitg1 = true;
      } else {
        i++;
      }
    }
  }

  Linear(fv0, z, fv1);
  f0 = 0.0F;
  for (i = 0; i < 9; i++) {
    f0 += fv1[i] * (float)b_alphas[i];
  }

  scores[iloc[1] - 1] = f0 + -0.385483027F;
  k = 0;
  if (scores[1] > scores[0]) {
    k = 1;
  }

  loop_ub = -2 * k;
  label[0].f1.size[0] = 1;
  label[0].f1.size[1] = loop_ub + 7;
  for (i = 0; i <= loop_ub + 6; i++) {
    label[0].f1.data[label[0].f1.size[0] * i] = CompactMdl_ClassNames[k + (i <<
      1)];
  }
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void predictFault_initialize(void)
{
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void predictFault_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for predictFault.c
 *
 * [EOF]
 */
