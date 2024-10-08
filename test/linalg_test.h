#include <tau/tau.h>
#include "../src/include/linalg.h"

TEST(linalg, m_transpose){
    gsl_matrix *A = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(A, 0, 0, 1);
    gsl_matrix_set(A, 0, 1, 2);
    gsl_matrix_set(A, 0, 2, 3);
    gsl_matrix_set(A, 1, 0, 4);
    gsl_matrix_set(A, 1, 1, 5);
    gsl_matrix_set(A, 1, 2, 6);
    
    gsl_matrix *A_t = gsl_matrix_alloc(3, 2);
    gsl_matrix_set(A_t, 0, 0, 1);
    gsl_matrix_set(A_t, 1, 0, 2);
    gsl_matrix_set(A_t, 2, 0, 3);
    gsl_matrix_set(A_t, 0, 1, 4);
    gsl_matrix_set(A_t, 1, 1, 5);
    gsl_matrix_set(A_t, 2, 1, 6);
    
    gsl_matrix *A_t_test = gsl_matrix_alloc(3, 2);
    gsl_matrix_transpose_memcpy(A_t_test, A);
    
    for(int i = 0; i < A_t->size1; i++){
        for(int j = 0; j < A_t->size2; j++){
            REQUIRE_EQ(gsl_matrix_get(A_t, i, j), gsl_matrix_get(A_t_test, i, j));
        }
    }
    
    gsl_matrix_free(A);
    gsl_matrix_free(A_t);
    gsl_matrix_free(A_t_test);
}

TEST(linalg, mm_multiply){
    gsl_matrix *A = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(A, 0, 0, 1);
    gsl_matrix_set(A, 0, 1, 2);
    gsl_matrix_set(A, 0, 2, 3);
    gsl_matrix_set(A, 1, 0, 4);
    gsl_matrix_set(A, 1, 1, 5);
    gsl_matrix_set(A, 1, 2, 6);

    gsl_matrix *B = gsl_matrix_alloc(3, 2);
    gsl_matrix_set(B, 0, 0, 1);
    gsl_matrix_set(B, 0, 1, 2);
    gsl_matrix_set(B, 1, 0, 3);
    gsl_matrix_set(B, 1, 1, 4);
    gsl_matrix_set(B, 2, 0, 5);
    gsl_matrix_set(B, 2, 1, 6);

    gsl_matrix *C = gsl_matrix_alloc(2, 2);
    gsl_matrix_set(C, 0, 0, 22);
    gsl_matrix_set(C, 0, 1, 28);
    gsl_matrix_set(C, 1, 0, 49);
    gsl_matrix_set(C, 1, 1, 64);

    gsl_matrix *C_test = mm_multiply(A, B);

    for(int i = 0; i < C->size1; i++){
        for(int j = 0; j < C->size2; j++){
            REQUIRE_EQ(gsl_matrix_get(C, i, j), gsl_matrix_get(C_test, i, j));
        }
    }

    gsl_matrix_free(A);
    gsl_matrix_free(B);
    gsl_matrix_free(C);
}

TEST(linalg, mv_multiply){
    gsl_matrix *A = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(A, 0, 0, 1);
    gsl_matrix_set(A, 0, 1, 2);
    gsl_matrix_set(A, 0, 2, 3);
    gsl_matrix_set(A, 1, 0, 4);
    gsl_matrix_set(A, 1, 1, 5);
    gsl_matrix_set(A, 1, 2, 6);

    gsl_vector *v = gsl_vector_alloc(3);
    gsl_vector_set(v, 0, 1);
    gsl_vector_set(v, 1, 2);
    gsl_vector_set(v, 2, 3);

    gsl_vector *u = gsl_vector_alloc(2);
    gsl_vector_set(u, 0, 14);
    gsl_vector_set(u, 1, 32);

    gsl_vector *u_test = mv_multiply(A, v);

    for(int i = 0; i < u->size; i++){
        REQUIRE_EQ(gsl_vector_get(u, i), gsl_vector_get(u_test, i));
    }

    gsl_matrix_free(A);
    gsl_vector_free(v);
    gsl_vector_free(u);

}

TEST(linalg, sm_multiply){
    gsl_matrix *A = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(A, 0, 0, 1);
    gsl_matrix_set(A, 0, 1, 2);
    gsl_matrix_set(A, 0, 2, 3);
    gsl_matrix_set(A, 1, 0, 4);
    gsl_matrix_set(A, 1, 1, 5);
    gsl_matrix_set(A, 1, 2, 6);

    gsl_matrix *B = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(B, 0, 0, 2);
    gsl_matrix_set(B, 0, 1, 4);
    gsl_matrix_set(B, 0, 2, 6);
    gsl_matrix_set(B, 1, 0, 8);
    gsl_matrix_set(B, 1, 1, 10);
    gsl_matrix_set(B, 1, 2, 12);

    gsl_matrix *B_test = sm_multiply(2, A);

    for(int i = 0; i < B->size1; i++){
        for(int j = 0; j < B->size2; j++){
            REQUIRE_EQ(gsl_matrix_get(B, i, j), gsl_matrix_get(B_test, i, j));
        }
    }

    gsl_matrix_free(A);
    gsl_matrix_free(B);
}

TEST(linalg, mm_add){
    gsl_matrix *A = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(A, 0, 0, 1);
    gsl_matrix_set(A, 0, 1, 2);
    gsl_matrix_set(A, 0, 2, 3);
    gsl_matrix_set(A, 1, 0, 4);
    gsl_matrix_set(A, 1, 1, 5);
    gsl_matrix_set(A, 1, 2, 6);

    gsl_matrix *B = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(B, 0, 0, 2);
    gsl_matrix_set(B, 0, 1, 4);
    gsl_matrix_set(B, 0, 2, 6);
    gsl_matrix_set(B, 1, 0, 8);
    gsl_matrix_set(B, 1, 1, 10);
    gsl_matrix_set(B, 1, 2, 12);

    gsl_matrix *C = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(C, 0, 0, 3);
    gsl_matrix_set(C, 0, 1, 6);
    gsl_matrix_set(C, 0, 2, 9);
    gsl_matrix_set(C, 1, 0, 12);
    gsl_matrix_set(C, 1, 1, 15);
    gsl_matrix_set(C, 1, 2, 18);

    gsl_matrix *C_test = mm_add(A, B);

    for(int i = 0; i < C->size1; i++){
        for(int j = 0; j < C->size2; j++){
            REQUIRE_EQ(gsl_matrix_get(C, i, j), gsl_matrix_get(C_test, i, j));
        }
    }

    gsl_matrix_free(A);
    gsl_matrix_free(B);
    gsl_matrix_free(C);
}

TEST(linalg, vv_subtract){
    gsl_vector *v = gsl_vector_alloc(3);
    gsl_vector_set(v, 0, 1);
    gsl_vector_set(v, 1, 2);
    gsl_vector_set(v, 2, 3);

    gsl_vector *w = gsl_vector_alloc(3);
    gsl_vector_set(w, 0, 4);
    gsl_vector_set(w, 1, 5);
    gsl_vector_set(w, 2, 6);

    gsl_vector *u = gsl_vector_alloc(3);
    gsl_vector_set(u, 0, -3);
    gsl_vector_set(u, 1, -3);
    gsl_vector_set(u, 2, -3);

    gsl_vector *u_test = vv_subtract(v, w);

    for(int i = 0; i < u->size; i++){
        REQUIRE_EQ(gsl_vector_get(u, i), gsl_vector_get(u_test, i));
    }

    gsl_vector_free(v);
    gsl_vector_free(w);
    gsl_vector_free(u);
}

TEST(linalg, m_pseudoinverse){
    gsl_matrix *A = gsl_matrix_alloc(2, 2);
    gsl_matrix_set(A, 0, 0, 1);
    gsl_matrix_set(A, 0, 1, 0);
    gsl_matrix_set(A, 1, 0, 1);
    gsl_matrix_set(A, 1, 1, 0);
    // print_matrix(A);
    gsl_matrix *A_pinv = gsl_matrix_alloc(2, 2);
    gsl_matrix_set(A_pinv, 0, 0, 0.5);
    gsl_matrix_set(A_pinv, 0, 1, 0.5);
    gsl_matrix_set(A_pinv, 1, 0, 0);
    gsl_matrix_set(A_pinv, 1, 1, 0);

    gsl_matrix *A_pinv_test = m_pseudoinverse(A);

    for(int i = 0; i < A_pinv->size1; i++){
        for(int j = 0; j < A_pinv->size2; j++){
            REQUIRE_LT(fabs(gsl_matrix_get(A_pinv, i, j) - gsl_matrix_get(A_pinv_test, i, j)), 1e-10);
        }
    }

    // Test another case
    gsl_matrix_set(A, 0, 0, 1);
    gsl_matrix_set(A, 0, 1, 1);
    gsl_matrix_set(A, 1, 0, 1);
    gsl_matrix_set(A, 1, 1, 1);

    gsl_matrix_set(A_pinv_test, 0, 0, 0.25);
    gsl_matrix_set(A_pinv_test, 0, 1, 0.25);
    gsl_matrix_set(A_pinv_test, 1, 0, 0.25);
    gsl_matrix_set(A_pinv_test, 1, 1, 0.25);

    A_pinv = m_pseudoinverse(A);

    for(int i = 0; i < A_pinv->size1; i++){
        for(int j = 0; j < A_pinv->size2; j++){
            REQUIRE_LT(fabs(gsl_matrix_get(A_pinv, i, j) - gsl_matrix_get(A_pinv_test, i, j)), 1e-10);
        }
    }


    gsl_matrix_free(A);
    gsl_matrix_free(A_pinv);
    gsl_matrix_free(A_pinv_test);

    // One last test
    gsl_matrix *C = gsl_matrix_alloc(2, 3);
    gsl_matrix_set(C, 0, 0, 1);
    gsl_matrix_set(C, 0, 1, 0);
    gsl_matrix_set(C, 0, 2, 0);
    gsl_matrix_set(C, 1, 0, 0);
    gsl_matrix_set(C, 1, 1, 1);
    gsl_matrix_set(C, 1, 2, 1);

    gsl_matrix *C_pinv_test = gsl_matrix_alloc(3, 2);
    gsl_matrix_set(C_pinv_test, 0, 0, 1);
    gsl_matrix_set(C_pinv_test, 0, 1, 0);
    gsl_matrix_set(C_pinv_test, 1, 0, 0);
    gsl_matrix_set(C_pinv_test, 1, 1, 0.5);
    gsl_matrix_set(C_pinv_test, 2, 0, 0);
    gsl_matrix_set(C_pinv_test, 2, 1, 0.5);

    gsl_matrix *C_pinv = m_pseudoinverse(C);

    for(int i = 0; i < C_pinv->size1; i++){
        for(int j = 0; j < C_pinv->size2; j++){
            REQUIRE_LT(fabs(gsl_matrix_get(C_pinv, i, j) - gsl_matrix_get(C_pinv_test, i, j)), 1e-10);
        }
    }

    gsl_matrix_free(C);
    gsl_matrix_free(C_pinv);
    gsl_matrix_free(C_pinv_test);


}