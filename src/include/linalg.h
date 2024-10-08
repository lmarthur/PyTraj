#ifndef LINALG_H
#define LINALG_H

#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_blas.h>

void print_matrix(gsl_matrix *A){
    /*
    This function prints a matrix to the console.

    INPUTS:
    ----------------
        A (gsl_matrix *): The matrix to be printed.

    */

    for(int i = 0; i < A->size1; i++){
        for(int j = 0; j < A->size2; j++){
            printf("%f ", gsl_matrix_get(A, i, j));
        }
        printf("\n");
    }
}

void print_vector(gsl_vector *v){
    /*
    This function prints a vector to the console.

    INPUTS:
    ----------------
        v (gsl_vector *): The vector to be printed.

    */

    for(int i = 0; i < v->size; i++){
        printf("%f\n", gsl_vector_get(v, i));
    }
}

gsl_matrix *m_transpose(gsl_matrix *A){
    /*
    This function returns the transpose of a matrix.

    INPUTS:
    ----------------
        A (gsl_matrix *): The matrix to be transposed.

    OUTPUTS:
    ----------------
        A_t (gsl_matrix *): The transpose of A.

    */

    gsl_matrix *A_t = gsl_matrix_alloc(A->size2, A->size1);
    gsl_matrix_transpose_memcpy(A_t, A);
    return A_t;
}

gsl_matrix *mm_multiply(gsl_matrix *A, gsl_matrix *B){
    /*
    This function returns the product of two matrices.

    INPUTS:
    ----------------
        A (gsl_matrix *): The first matrix.
        B (gsl_matrix *): The second matrix.

    OUTPUTS:
    ----------------
        C (gsl_matrix *): The product of A and B.

    */

    gsl_matrix *C = gsl_matrix_alloc(A->size1, B->size2);
    gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1.0, A, B, 0.0, C);
    return C;
}

gsl_vector *mv_multiply(gsl_matrix *A, gsl_vector *v){
    /*
    This function returns the product of a matrix and a vector.

    INPUTS:
    ----------------
        A (gsl_matrix *): The matrix.
        v (gsl_vector *): The vector.

    OUTPUTS:
    ----------------
        w (gsl_vector *): The product of A and v.

    */

    gsl_vector *w = gsl_vector_alloc(A->size1);
    gsl_blas_dgemv(CblasNoTrans, 1.0, A, v, 0.0, w);
    return w;
}

gsl_matrix *sm_multiply(double s, gsl_matrix *A){
    /*
    This function returns the product of a scalar and a matrix.

    INPUTS:
    ----------------
        s (double): The scalar.
        A (gsl_matrix *): The matrix.

    OUTPUTS:
    ----------------
        B (gsl_matrix *): The product of s and A.

    */

    gsl_matrix *B = gsl_matrix_alloc(A->size1, A->size2);
    gsl_matrix_memcpy(B, A);
    gsl_matrix_scale(B, s);
    return B;
}

gsl_matrix *mm_add(gsl_matrix *A, gsl_matrix *B){
    /*
    This function returns the sum of two matrices.

    INPUTS:
    ----------------
        A (gsl_matrix *): The first matrix.
        B (gsl_matrix *): The second matrix.

    OUTPUTS:
    ----------------
        C (gsl_matrix *): The sum of A and B.

    */

    gsl_matrix *C = gsl_matrix_alloc(A->size1, A->size2);
    gsl_matrix_memcpy(C, A);
    gsl_matrix_add(C, B);
    return C;
}

gsl_vector *vv_subtract(gsl_vector *v, gsl_vector *w){
    /*
    This function returns the difference of two vectors.

    INPUTS:
    ----------------
        v (gsl_vector *): The first vector.
        w (gsl_vector *): The second vector.

    OUTPUTS:
    ----------------
        u (gsl_vector): The difference of v and w.

    */

    gsl_vector *u = gsl_vector_alloc(v->size);
    gsl_vector_memcpy(u, v);
    gsl_vector_sub(u, w);
    return u;
}

gsl_vector *vv_add(gsl_vector *v, gsl_vector *w){
    /*
    This function returns the sum of two vectors.

    INPUTS:
    ----------------
        v (gsl_vector *): The first vector.
        w (gsl_vector *): The second vector.

    OUTPUTS:
    ----------------
        u (gsl_vector): The sum of v and w.

    */

    gsl_vector *u = gsl_vector_alloc(v->size);
    gsl_vector_memcpy(u, v);
    gsl_vector_add(u, w);
    return u;
}

gsl_matrix *m_pseudoinverse(gsl_matrix *A){
    /*
    This function returns the pseudoinverse (also known as the Moore-Penrose inverse) of a matrix.
    Based on turingbirds/moore_penrose_pseudoinverse.c
    https://gist.github.com/turingbirds/5e99656e08dbe1324c99


    INPUTS:
    ----------------
        A (gsl_matrix *): The matrix.

    OUTPUTS:
    ----------------
        A_pinv (gsl_matrix *): The pseudoinverse of A.

    */

    gsl_matrix *V, *Sigma_pinv, *U, *A_pinv;
	gsl_matrix *_tmp_mat = NULL;
	gsl_vector *_tmp_vec;
	gsl_vector *u;
	double x, cutoff;
	size_t i, j;
	unsigned int n = A->size1;
	unsigned int m = A->size2;
	int was_swapped = 0;
    double rcond = 1e-10;

	if (m > n) {
		/* libgsl SVD can only handle the case m <= n - transpose matrix */
		was_swapped = 1;
		_tmp_mat = gsl_matrix_alloc(m, n);
		gsl_matrix_transpose_memcpy(_tmp_mat, A);
		A = _tmp_mat;
		i = m;
		m = n;
		n = i;
	}

	/* do SVD */
	V = gsl_matrix_alloc(m, m);
	u = gsl_vector_alloc(m);
	_tmp_vec = gsl_vector_alloc(m);
	gsl_linalg_SV_decomp(A, V, u, _tmp_vec);
	gsl_vector_free(_tmp_vec);

	/* compute Σ⁻¹ */
	Sigma_pinv = gsl_matrix_alloc(m, n);
	gsl_matrix_set_zero(Sigma_pinv);
	cutoff = rcond * gsl_vector_max(u);

	for (i = 0; i < m; ++i) {
		if (gsl_vector_get(u, i) > cutoff) {
			x = 1. / gsl_vector_get(u, i);
		}
		else {
			x = 0.;
		}
		gsl_matrix_set(Sigma_pinv, i, i, x);
	}

	/* libgsl SVD yields "thin" SVD - pad to full matrix by adding zeros */
	U = gsl_matrix_alloc(n, n);
	gsl_matrix_set_zero(U);

	for (i = 0; i < n; ++i) {
		for (j = 0; j < m; ++j) {
			gsl_matrix_set(U, i, j, gsl_matrix_get(A, i, j));
		}
	}

	if (_tmp_mat != NULL) {
		gsl_matrix_free(_tmp_mat);
	}

	/* two dot products to obtain pseudoinverse */
	_tmp_mat = gsl_matrix_alloc(m, n);
	gsl_blas_dgemm(CblasNoTrans, CblasNoTrans, 1., V, Sigma_pinv, 0., _tmp_mat);

	if (was_swapped) {
		A_pinv = gsl_matrix_alloc(n, m);
		gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1., U, _tmp_mat, 0., A_pinv);
	}
	else {
		A_pinv = gsl_matrix_alloc(m, n);
		gsl_blas_dgemm(CblasNoTrans, CblasTrans, 1., _tmp_mat, U, 0., A_pinv);
	}

	gsl_matrix_free(_tmp_mat);
	gsl_matrix_free(U);
	gsl_matrix_free(Sigma_pinv);
	gsl_vector_free(u);
	gsl_matrix_free(V);

	return A_pinv;

    return A_pinv;
}
#endif 