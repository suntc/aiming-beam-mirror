#include "deconvolveprocess.h"
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <QMessageBox>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <boost/iterator/counting_iterator.hpp>

using namespace std;
using namespace cv;

DeconvolveProcess::DeconvolveProcess()
{

}

double DeconvolveProcess::Laguerre_alphaval(int LaguerreLength, int &LG_order)
{
    // Reads the lookup table of alpha values and finds the one corresponding to the current data length and Laguerre order

    double alpha = 0.0;

    string line;
    int rows,cols;
    double my_array[1802][12];

    ifstream pFile ("./AlphaLookup/LG_alpha_vals.txt");
    if (pFile.is_open())
    {
        rows = 0;

        while(!pFile.eof())
        {
            while(getline(pFile, line))
            {
                cols=0;
                stringstream ss(line);
                while(ss >> my_array[rows][cols])
                {
                    cols++;
                }
                rows++;
            }
        }
        pFile.close();

        int col_num = 0;
        int row_num = 0;

        for (int i = 1; i < 12; i++)
        {
            if (my_array[0][i] == my_array[0][0])
            {
                col_num = i;
            }
        }

        for (int i = 1; i < 1802; i++)
        {
            if (my_array[i][0] == LaguerreLength)
            {
                row_num = i;
            }
        }

        if ((col_num != 0) || (row_num != 0))
        {
            alpha = my_array[row_num][col_num];
        }
        else
        {
            alpha = 0;
        }

        LG_order = my_array[0][0];
    }
    else
    {
        QMessageBox messageBox;
        messageBox.critical(0, "Error", "Unable to open LG alpha value lookup table file!");
        messageBox.setFixedSize(500, 200);

        LG_order = 0;
    }
    return alpha;
}

Mat DeconvolveProcess::Laguerre(int LaguerreOrder, int LaguerreLength, double alpha)
{
    // See Jing's paper for this, symbols are the same

    Mat L(LaguerreLength, LaguerreOrder, CV_64F);
    for (int j = 0; j < LaguerreOrder; j++)
    {
        if (j == 0)
        {
            for (int i = 0; i < LaguerreLength; i++)
            {
                L.row(i).col(j) = sqrt((pow(alpha, i)) * (1 - alpha));
            }
        }
        else
        {
            for (int i = 0; i < LaguerreLength; i++)
            {
                if (i == 0)
                {
                    L.row(i).col(j) = sqrt(alpha) * L.row(i).col(j - 1);
                }
                else
                {
                    L.row(i).col(j) = (sqrt(alpha) * (L.row(i - 1).col(j) + L.row(i).col(j - 1)) - L.row(i - 1).col(j - 1));
                }
            }
        }
    }
    return L;
}

Mat DeconvolveProcess::LaguerreFilt(vector<double> laser, Mat &L)
{
    // See Jing's paper for this, symbols are the same

    Mat vv_trans, vv;

    vector<double> laser_trans;
    flip(laser, laser_trans, 1);
    Mat padded;
    copyMakeBorder(L, padded, (round(L.rows / 2) - 1), 0, 0, 0, BORDER_CONSTANT, Scalar(0));
    filter2D(padded.t(), vv_trans, -1, laser_trans, Point(-1, -1), 0, BORDER_CONSTANT);
    vv_trans = vv_trans.t();

    Rect myROI(0, 0, L.cols, L.rows);
    vv = vv_trans(myROI);

    return vv;
}

Mat DeconvolveProcess::deriv3rd(int DataLength, Mat &L)
{
    // See Jing's paper for this, symbols are the same

    Mat idMat = Mat::eye(DataLength, DataLength, CV_64FC1);
    Mat D_Mat, D_Mat_filt;
    vector<double> filtVector;
    filtVector.push_back(1);
    filtVector.push_back(-3);
    filtVector.push_back(3);
    filtVector.push_back(-1);

    Mat padded2;
    copyMakeBorder(idMat, padded2, 1, 0, 0, 0, BORDER_CONSTANT, Scalar(0));
    filter2D(padded2.t(), D_Mat_filt, -1, filtVector, Point(-1, -1), 0, BORDER_CONSTANT);
    D_Mat_filt = D_Mat_filt.t();
    Rect myROI0(0, 0,(int) (idMat.cols - (filtVector.size() - 1)), idMat.rows);
    Mat D_Mat_filt0 = D_Mat_filt(myROI0);

    D_Mat = D_Mat_filt0.t() * L;

    return D_Mat;
}

Mat DeconvolveProcess::CholFact(Mat &H)
{
    // See Jing's paper for information on these, symbols are the same

    int n = (int)H.rows;
    Mat H_chol(H.rows, H.cols, H.type());
    H_chol.setTo(0.0);
    for (int i = 0; i < n; i++)
    {
        Mat LL;
        LL = H_chol.row(i);
        LL = LL.t();
        sqrt((H.row(i).col(i) - H_chol.row(i) * LL), H_chol.row(i).col(i));

        for (int j= (i + 1); j < n; j++)
        {
            Mat LH_chol;
            LH_chol = H_chol.row(j);
            LH_chol = LH_chol.t();
            H_chol.row(j).col(i) = (H.row(j).col(i) - H_chol.row(i) * LH_chol) / H_chol.row(i).col(i);
        }
    }

    H_chol = H_chol.t();

    return H_chol;
}

void DeconvolveProcess::preMatChannels(vector<double> laser, deconMats &deconMatrices_CH1, int DataLength, int LaguerreOrder, double alpha)
{
    DeconvolveProcess * allfuncs = new DeconvolveProcess;

    // Laguerre
    deconMatrices_CH1.L = allfuncs->Laguerre(LaguerreOrder, DataLength, alpha);
    Mat L;
    deconMatrices_CH1.L.copyTo(L);

    // Filtered Laguerre
    deconMatrices_CH1.V = allfuncs->LaguerreFilt(laser, L);

    // Third order derivative non-negative constraints
    deconMatrices_CH1.D = allfuncs->deriv3rd(DataLength, L);

    // H matrix
    mulTransposed(deconMatrices_CH1.V, deconMatrices_CH1.H, true);
    Mat H1 = deconMatrices_CH1.H.inv();

    // Cholesky Factorization
    Mat H_chol = CholFact(H1);

    // Rest Matrices
    Mat C;
    C = H_chol * deconMatrices_CH1.D.t();
    int Ccols = C.cols;

    deconMatrices_CH1.Lambda = H_chol * deconMatrices_CH1.V.t();

    // NNLS Matrices
    for (int i = 0; i < Ccols; i++)
    {
        for (int j = 0; j < LaguerreOrder; j++)
        {
            deconMatrices_CH1.B.push_back(C.at<double>(j, i));
        }
    }

    delete allfuncs;
}

double DeconvolveProcess::lifetCalc(deconMats deconMatrices, Mat fIRF1, int LaguerreOrder, int Ccols, double resTime)
{
    double* A = &deconMatrices.B[0];
    Mat lam = DeconvolveProcess::solveNNLS(fIRF1, deconMatrices.Lambda, LaguerreOrder, Ccols, A);

    // Laguerre Coefs
    Mat cc;
    Mat temp;
    temp = deconMatrices.V.t() * fIRF1 - deconMatrices.D.t() * lam;
    solve(deconMatrices.H, temp, cc);

    // Decay
    Mat h;
    h = deconMatrices.L * cc;

    // Lifetime
    Mat lifet;
    vector<double> ivec(boost::counting_iterator<int>(0), boost::counting_iterator<int>(h.rows));
    Mat dtime(ivec);

    lifet = ((0.5 + dtime.t()) * h) / sum(h).val[0];

    double lifet_out = lifet.at<double>(0, 0) * resTime;
    lifet_out = round(lifet_out * 10) / 10;

    return lifet_out;
}

Mat DeconvolveProcess::solveNNLS(Mat fIRF1, Mat l1, int LaguerreOrder, int Ccols, double* AA)
{
    // See Jing's paper for this, symbols are the same

    Mat d = l1 * fIRF1;

    // NNLS
    int mda = LaguerreOrder;
    int m = LaguerreOrder, nn = Ccols;
    double *b = new double[d.rows + 1];
    int k = 0;

    for (int i = 0; i < d.rows; i++)
    {
        b[k] = d.at<double>(i, 0);
        k++;
    }
    b[k] = 1;

    double *lam_array = new double[Ccols + 1];
    fill_n(lam_array, Ccols + 1, 1.0);
    double rnorm;
    double *w = new double[Ccols + 1];
    double *zz = new double[Ccols + 1];
    int *indx = new int[Ccols + 2];

    int mode;

    nnls(AA, mda, m, nn, b, lam_array, &rnorm, w, zz, indx, &mode);

    Mat lam(Ccols, 1, CV_64FC1, lam_array);

    Mat lam2 = lam.clone();
    return lam2;
}

double d_sign(double& a, double& b)
{
  double x;
  x = (a >= 0 ? a : - a);
  return (b >= 0 ? x : -x);
}

/* Table of constant values */

int c__1 = 1;
int c__0 = 0;
int c__2 = 2;


int DeconvolveProcess::nnls(double* a,  int mda,  int m,  int n, double* b,
     double* x, double* rnorm, double* w, double* zz, int* index,
     int* mode)
{
  /* System generated locals */
  int a_dim1, a_offset, idx1, idx2;
  double d1, d2;


  /* Local variables */
  static int iter;
  static double temp, wmax;
  static int i__, j, l;
  static double t, alpha, asave;
  static int itmax, izmax, nsetp;
  static double unorm, ztest, cc;
  double dummy[2];
  static int ii, jj, ip;
  static double sm;
  static int iz, jz;
  static double up, ss;
  static int rtnkey, iz1, iz2, npp1;

  /*     ------------------------------------------------------------------
   */
  /*     int INDEX(N) */
  /*     double precision A(MDA,N), B(M), W(N), X(N), ZZ(M) */
  /*     ------------------------------------------------------------------
   */
  /* Parameter adjustments */
  a_dim1 = mda;
  a_offset = a_dim1 + 1;
  a -= a_offset;
  --b;
  --x;
  --w;
  --zz;
  --index;

  /* Function Body */
  *mode = 1;
  if (m <= 0 || n <= 0) {
    *mode = 2;
    return 0;
  }
  iter = 0;
  itmax = n * 3;

  /*                    INITIALIZE THE ARRAYS INDEX() AND X(). */

  idx1 = n;
  for (i__ = 1; i__ <= idx1; ++i__) {
    x[i__] = 0.;
    /* L20: */
    index[i__] = i__;
  }

  iz2 = n;
  iz1 = 1;
  nsetp = 0;
  npp1 = 1;
  /*                             ******  MAIN LOOP BEGINS HERE  ****** */
 L30:
  /*                  QUIT IF ALL COEFFICIENTS ARE ALREADY IN THE SOLUTION.
   */
  /*                        OR IF M COLS OF A HAVE BEEN TRIANGULARIZED. */

  if (iz1 > iz2 || nsetp >= m) {
    goto L350;
  }

  /*         COMPUTE COMPONENTS OF THE DUAL (NEGATIVE GRADIENT) VECTOR W().
   */

  idx1 = iz2;
  for (iz = iz1; iz <= idx1; ++iz) {
    j = index[iz];
    sm = 0.;
    idx2 = m;
    for (l = npp1; l <= idx2; ++l) {
      /* L40: */
      sm += a[l + j * a_dim1] * b[l];
    }
    w[j] = sm;
    /* L50: */
  }
  /*                                   FIND LARGEST POSITIVE W(J). */
 L60:
  wmax = 0.;
  idx1 = iz2;
  for (iz = iz1; iz <= idx1; ++iz) {
    j = index[iz];
    if (w[j] > wmax) {
      wmax = w[j];
      izmax = iz;
    }
    /* L70: */
  }

  /*             IF WMAX .LE. 0. GO TO TERMINATION. */
  /*             THIS INDICATES SATISFACTION OF THE KUHN-TUCKER CONDITIONS.
   */

  if (wmax <= 0.) {
    goto L350;
  }
  iz = izmax;
  j = index[iz];

  /*     THE SIGN OF W(J) IS OK FOR J TO BE MOVED TO SET P. */
  /*     BEGIN THE TRANSFORMATION AND CHECK NEW DIAGONAL ELEMENT TO AVOID */
  /*     NEAR LINEAR DEPENDENCE. */

  asave = a[npp1 + j * a_dim1];
  idx1 = npp1 + 1;
  DeconvolveProcess::h12(c__1, &npp1, &idx1, m, &a[j * a_dim1 + 1], &c__1, &up, dummy, &
      c__1, &c__1, &c__0);
  unorm = 0.;
  if (nsetp != 0) {
    idx1 = nsetp;
    for (l = 1; l <= idx1; ++l) {
      /* L90: */
      /* Computing 2nd power */
      d1 = a[l + j * a_dim1];
      unorm += d1 * d1;
    }
  }
  unorm = sqrt(unorm);
  d2 = unorm + (d1 = a[npp1 + j * a_dim1], nnls_abs(d1)) * .01;
  if ((d2 - unorm) > 0.) {

    /*        COL J IS SUFFICIENTLY INDEPENDENT.  COPY B INTO ZZ, UPDATE Z
          Z */
    /*        AND SOLVE FOR ZTEST ( = PROPOSED NEW VALUE FOR X(J) ). */

    idx1 = m;
    for (l = 1; l <= idx1; ++l) {
      /* L120: */
      zz[l] = b[l];
    }
    idx1 = npp1 + 1;
    h12(c__2, &npp1, &idx1, m, &a[j * a_dim1 + 1], &c__1, &up, (zz+1), &
    c__1, &c__1, &c__1);
    ztest = zz[npp1] / a[npp1 + j * a_dim1];

    /*                                     SEE IF ZTEST IS POSITIVE */

    if (ztest > 0.) {
      goto L140;
    }
  }

  /*     REJECT J AS A CANDIDATE TO BE MOVED FROM SET Z TO SET P. */
  /*     RESTORE A(NPP1,J), SET W(J)=0., AND LOOP BACK TO TEST DUAL */
  /*     COEFFS AGAIN. */

  a[npp1 + j * a_dim1] = asave;
  w[j] = 0.;
  goto L60;

  /*     THE INDEX  J=INDEX(IZ)  HAS BEEN SELECTED TO BE MOVED FROM */
  /*     SET Z TO SET P.    UPDATE B,  UPDATE INDICES,  APPLY HOUSEHOLDER */
  /*     TRANSFORMATIONS TO COLS IN NEW SET Z,  ZERO SUBDIAGONAL ELTS IN */
  /*     COL J,  SET W(J)=0. */

 L140:
  idx1 = m;
  for (l = 1; l <= idx1; ++l) {
    /* L150: */
    b[l] = zz[l];
  }

  index[iz] = index[iz1];
  index[iz1] = j;
  ++iz1;
  nsetp = npp1;
  ++npp1;

  if (iz1 <= iz2) {
    idx1 = iz2;
    for (jz = iz1; jz <= idx1; ++jz) {
      jj = index[jz];
      h12(c__2, &nsetp, &npp1, m,
      &a[j * a_dim1 + 1], &c__1, &up,
      &a[jj * a_dim1 + 1], &c__1, &mda, &c__1);
      /* L160: */
    }
  }

  if (nsetp != m) {
    idx1 = m;
    for (l = npp1; l <= idx1; ++l) {
      /* L180: */
      // SS: CHECK THIS DAMAGE....
      a[l + j * a_dim1] = 0.;
    }
  }

  w[j] = 0.;
  /*                                SOLVE THE TRIANGULAR SYSTEM. */
  /*                                STORE THE SOLUTION TEMPORARILY IN ZZ().
   */
  rtnkey = 1;
  goto L400;
 L200:

  /*                       ******  SECONDARY LOOP BEGINS HERE ****** */

  /*                          ITERATION COUNTER. */

 L210:
  ++iter;
  if (iter > itmax) {
    *mode = 3;
    /* The following lines were replaced after the f2c translation */
    /* s_wsfe(&io___22); */
    /* do_fio(&c__1, " NNLS quitting on iteration count.", 34L); */
    /* e_wsfe(); */
    fprintf(stdout, "\n NNLS quitting on iteration count.\n");
    fflush(stdout);
    goto L350;
  }

  /*                    SEE IF ALL NEW CONSTRAINED COEFFS ARE FEASIBLE. */
  /*                                  IF NOT COMPUTE ALPHA. */

  alpha = 2.;
  idx1 = nsetp;
  for (ip = 1; ip <= idx1; ++ip) {
    l = index[ip];
    if (zz[ip] <= 0.) {
      t = -x[l] / (zz[ip] - x[l]);
      if (alpha > t) {
    alpha = t;
    jj = ip;
      }
    }
    /* L240: */
  }

  /*          IF ALL NEW CONSTRAINED COEFFS ARE FEASIBLE THEN ALPHA WILL */
  /*          STILL = 2.    IF SO EXIT FROM SECONDARY LOOP TO MAIN LOOP. */

  if (alpha == 2.) {
    goto L330;
  }

  /*          OTHERWISE USE ALPHA WHICH WILL BE BETWEEN 0. AND 1. TO */
  /*          INTERPOLATE BETWEEN THE OLD X AND THE NEW ZZ. */

  idx1 = nsetp;
  for (ip = 1; ip <= idx1; ++ip) {
    l = index[ip];
    x[l] += alpha * (zz[ip] - x[l]);
    /* L250: */
  }

  /*        MODIFY A AND B AND THE INDEX ARRAYS TO MOVE COEFFICIENT I */
  /*        FROM SET P TO SET Z. */

  i__ = index[jj];
 L260:
  x[i__] = 0.;

  if (jj != nsetp) {
    ++jj;
    idx1 = nsetp;
    for (j = jj; j <= idx1; ++j) {
      ii = index[j];
      index[j - 1] = ii;
      DeconvolveProcess::g1(&a[j - 1 + ii * a_dim1], &a[j + ii * a_dim1],
     &cc, &ss, &a[j - 1 + ii * a_dim1]);
      // SS: CHECK THIS DAMAGE...
      a[j + ii * a_dim1] = 0.;
      idx2 = n;
      for (l = 1; l <= idx2; ++l) {
    if (l != ii) {

      /*                 Apply procedure G2 (CC,SS,A(J-1,L),A(J,
                 L)) */

      temp = a[j - 1 + l * a_dim1];
      // SS: CHECK THIS DAMAGE
      a[j - 1 + l * a_dim1] = cc * temp + ss * a[j + l * a_dim1];
      a[j + l * a_dim1] = -ss * temp + cc * a[j + l * a_dim1];
    }
    /* L270: */
      }

      /*                 Apply procedure G2 (CC,SS,B(J-1),B(J)) */

      temp = b[j - 1];
      b[j - 1] = cc * temp + ss * b[j];
      b[j] = -ss * temp + cc * b[j];
      /* L280: */
    }
  }

  npp1 = nsetp;
  --nsetp;
  --iz1;
  index[iz1] = i__;

  /*        SEE IF THE REMAINING COEFFS IN SET P ARE FEASIBLE.  THEY SHOULD
   */
  /*        BE BECAUSE OF THE WAY ALPHA WAS DETERMINED. */
  /*        IF ANY ARE INFEASIBLE IT IS DUE TO ROUND-OFF ERROR.  ANY */
  /*        THAT ARE NONPOSITIVE WILL BE SET TO ZERO */
  /*        AND MOVED FROM SET P TO SET Z. */

  idx1 = nsetp;
  for (jj = 1; jj <= idx1; ++jj) {
    i__ = index[jj];
    if (x[i__] <= 0.) {
      goto L260;
    }
    /* L300: */
  }

  /*         COPY B( ) INTO ZZ( ).  THEN SOLVE AGAIN AND LOOP BACK. */

  idx1 = m;
  for (i__ = 1; i__ <= idx1; ++i__) {
    /* L310: */
    zz[i__] = b[i__];
  }
  rtnkey = 2;
  goto L400;
 L320:
  goto L210;
  /*                      ******  END OF SECONDARY LOOP  ****** */

 L330:
  idx1 = nsetp;
  for (ip = 1; ip <= idx1; ++ip) {
    i__ = index[ip];
    /* L340: */
    x[i__] = zz[ip];
  }
  /*        ALL NEW COEFFS ARE POSITIVE.  LOOP BACK TO BEGINNING. */
  goto L30;

  /*                        ******  END OF MAIN LOOP  ****** */

  /*                        COME TO HERE FOR TERMINATION. */
  /*                     COMPUTE THE NORM OF THE FINAL RESIDUAL VECTOR. */

 L350:
  sm = 0.;
  if (npp1 <= m) {
    idx1 = m;
    for (i__ = npp1; i__ <= idx1; ++i__) {
      /* L360: */
      /* Computing 2nd power */
      d1 = b[i__];
      sm += d1 * d1;
    }
  } else {
    idx1 = n;
    for (j = 1; j <= idx1; ++j) {
      /* L380: */
      w[j] = 0.;
    }
  }
  *rnorm = sqrt(sm);
  return 0;

  /*     THE FOLLOWING BLOCK OF CODE IS USED AS AN INTERNAL SUBROUTINE */
  /*     TO SOLVE THE TRIANGULAR SYSTEM, PUTTING THE SOLUTION IN ZZ(). */

 L400:
  idx1 = nsetp;
  for (l = 1; l <= idx1; ++l) {
    ip = nsetp + 1 - l;
    if (l != 1) {
      idx2 = ip;
      for (ii = 1; ii <= idx2; ++ii) {
    zz[ii] -= a[ii + jj * a_dim1] * zz[ip + 1];
    /* L410: */
      }
    }
    jj = index[ip];
    zz[ip] /= a[ip + jj * a_dim1];
    /* L430: */
  }
  switch ((int)rtnkey) {
  case 1:  goto L200;
  case 2:  goto L320;
  }

  /* The next line was added after the f2c translation to keep
     compilers from complaining about a void return from a non-void
     function. */
  return 0;

} /* nnls_ */

int DeconvolveProcess::g1(double* a, double* b, double* cterm, double* sterm, double* sig)
{
  /* System generated locals */
  double d;
  static double xr, yr;

  if (nnls_abs(*a) > nnls_abs(*b)) {
    xr = *b / *a;
    /* Computing 2nd power */
    d = xr;
    yr = sqrt(d * d + 1.);
    d = 1. / yr;
    *cterm = d_sign(d, *a);
    *sterm = *cterm * xr;
    *sig = nnls_abs(*a) * yr;
    return 0;
  }
  if (*b != 0.) {
    xr = *a / *b;
    /* Computing 2nd power */
    d = xr;
    yr = sqrt(d * d + 1.);
    d = 1. / yr;
    *sterm = d_sign(d, *b);
    *cterm = *sterm * xr;
    *sig = nnls_abs(*b) * yr;
    return 0;
  }
  *sig = 0.;
  *cterm = 0.;
  *sterm = 1.;
  return 0;
} /* g1_ */


/* See nnls.h for explanation */
int DeconvolveProcess::h12(int mode, int* lpivot, int* l1,
    int m, double* u, int* iue, double* up, double* c__,
    int* ice, int* icv, int* ncv)
{
  /* System generated locals */
  int u_dim1, u_offset, idx1, idx2;
  double d, d2;

  /* Builtin functions */
  /* The following line was commented out after the f2c translation */
  /* double sqrt(); */

  /* Local variables */
  static int incr;
  static double b;
  static int i__, j;
  static double clinv;
  static int i2, i3, i4;
  static double cl, sm;

  /*     ------------------------------------------------------------------
   */
  /*     double precision U(IUE,M) */
  /*     ------------------------------------------------------------------
   */
  /* Parameter adjustments */
  u_dim1 = *iue;
  u_offset = u_dim1 + 1;
  u -= u_offset;
  --c__;

  /* Function Body */
  if (0 >= *lpivot || *lpivot >= *l1 || *l1 > m) {
    return 0;
  }
  cl = (d = u[*lpivot * u_dim1 + 1], nnls_abs(d));
  if (mode == 2) {
    goto L60;
  }
  /*                            ****** CONSTRUCT THE TRANSFORMATION. ******
   */
  idx1 = m;
  for (j = *l1; j <= idx1; ++j) {
    /* L10: */
    /* Computing MAX */
    d2 = (d = u[j * u_dim1 + 1], nnls_abs(d));
    cl = nnls_max(d2,cl);
  }
  if (cl <= 0.) {
    goto L130;
  } else {
    goto L20;
  }
 L20:
  clinv = 1. / cl;
  /* Computing 2nd power */
  d = u[*lpivot * u_dim1 + 1] * clinv;
  sm = d * d;
  idx1 = m;
  for (j = *l1; j <= idx1; ++j) {
    /* L30: */
    /* Computing 2nd power */
    d = u[j * u_dim1 + 1] * clinv;
    sm += d * d;
  }
  cl *= sqrt(sm);
  if (u[*lpivot * u_dim1 + 1] <= 0.) {
    goto L50;
  } else {
    goto L40;
  }
 L40:
  cl = -cl;
 L50:
  *up = u[*lpivot * u_dim1 + 1] - cl;
  u[*lpivot * u_dim1 + 1] = cl;
  goto L70;
  /*            ****** APPLY THE TRANSFORMATION  I+U*(U**T)/B  TO C. ******
   */

 L60:
  if (cl <= 0.) {
    goto L130;
  } else {
    goto L70;
  }
 L70:
  if (*ncv <= 0) {
    return 0;
  }
  b = *up * u[*lpivot * u_dim1 + 1];
  /*                       B  MUST BE NONPOSITIVE HERE.  IF B = 0., RETURN.
   */

  if (b >= 0.) {
    goto L130;
  } else {
    goto L80;
  }
 L80:
  b = 1. / b;
  i2 = 1 - *icv + *ice * (*lpivot - 1);
  incr = *ice * (*l1 - *lpivot);
  idx1 = *ncv;
  for (j = 1; j <= idx1; ++j) {
    i2 += *icv;
    i3 = i2 + incr;
    i4 = i3;
    sm = c__[i2] * *up;
    idx2 = m;
    for (i__ = *l1; i__ <= idx2; ++i__) {
      sm += c__[i3] * u[i__ * u_dim1 + 1];
      /* L90: */
      i3 += *ice;
    }
    if (sm != 0.) {
      goto L100;
    } else {
      goto L120;
    }
  L100:
    sm *= b;
    c__[i2] += sm * *up;
    idx2 = m;
    for (i__ = *l1; i__ <= idx2; ++i__) {
      c__[i4] += sm * u[i__ * u_dim1 + 1];
      /* L110: */
      i4 += *ice;
    }
  L120:
    ;
  }
 L130:
  return 0;
}
