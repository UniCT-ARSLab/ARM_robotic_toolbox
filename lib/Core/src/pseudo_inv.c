/*
 * pseudo_inv.c
 *
 *  Created on: Oct 13, 2026
 *      Author: corrado
 */


#include <math.h>
#include "manipulator.h"

float dot_product_columns(arm_matrix_instance_f32 * mat, int col1, int col2)
{
	float sum = 0.0;
    for (int i = 0; i < mat->numRows; i++) {
        sum += (MAT_EL(*mat, i, col1)) * (MAT_EL(*mat, i, col2));
    }
    return sum;
}

#define MAX_ITER 100    // Numero massimo di iterazioni per la convergenza
#define EPSILON 1e-9    // Soglia sotto la quale un valore singolare è considerato zero
#define TOL 1e-15       // Tolleranza per la convergenza delle rotazioni di Jacobi


void jacobe_pseudo_inverse(Manipulator * m)
{
	// copy jacobe to W
	int i;
	for (i = 0; i < m->jacobe.numCols * m->jacobe.numRows;i++)
		m->W.pData[i] = m->jacobe.pData[i];

	// V = I
	for (i = 0; i < m->V.numCols * m->V.numRows;i++)
		m->V.pData[i] = 0;
	for (i = 0; i < m->V.numRows;i++)
		MAT_EL(m->V, i, i) = 1.0;


    // Processo iterativo di Jacobi per ortogonalizzare le colonne di W
    for (int iter = 0; iter < MAX_ITER; iter++) {
    	float max_error = 0.0;
        int p, q;

        // Esamina tutte le coppie di colonne (p, q)
        for (p = 0; p < m->W.numCols - 1; p++) {
            for (q = p + 1; q < m->W.numCols; q++) {
            	float riga_pp = dot_product_columns(&m->W, p, p);
            	float riga_qq = dot_product_columns(&m->W, q, q);
            	float riga_pq = dot_product_columns(&m->W, p, q);

                max_error = fmax(max_error, fabs(riga_pq) / fmax(1.0, sqrt(riga_pp * riga_qq)));

                // Se le colonne sono già ortogonali, salta la rotazione
                if (fabs(riga_pq) < TOL) continue;

                // Calcolo dell'angolo di rotazione di Jacobi
                float tau = (riga_pp - riga_qq) / (2.0 * riga_pq);
                float t = 1.0 / (fabs(tau) + sqrt(1.0 + tau * tau));
                if (tau < 0) t = -t;

                float c = 1.0 / sqrt(1.0 + t * t);
                float s = t * c;

                // Applica la rotazione alle colonne p e q di W
                for (int i = 0; i < m->W.numRows; i++) {
                	float wp = MAT_EL(m->W, i, p);
                	float wq = MAT_EL(m->W, i, q);
                	MAT_EL(m->W, i, p) = c * wp + s * wq;
                	MAT_EL(m->W, i, q) = -s * wp + c * wq;
                }

                // Applica la stessa rotazione alle colonne p e q di V
                for (int i = 0; i < m->V.numRows; i++) {
                	float vp = MAT_EL(m->V, i, p);
                	float vq = MAT_EL(m->V, i, q);
                	MAT_EL(m->V, i, p) = c * vp + s * vq;
                	MAT_EL(m->V, i, q) = -s * vp + c * vq;
                }
            }
        }
        // Se l'errore di ortogonalità è sotto la tolleranza, l'algoritmo ha convertito
        if (max_error < TOL) break;
    }

    for (int j = 0; j < m->W.numCols; j++) {
    	float norm_sq = dot_product_columns(&m->W, j, j);
    	float sigma = sqrt(norm_sq);

        if (sigma > EPSILON) {
            m->S_inv[j] = 1.0 / sigma; // Calcoliamo direttamente 1 / sigma
            for (int i = 0; i < m->W.numRows; i++) {
            	MAT_EL(m->U, i, j) = MAT_EL(m->W, i, j);
            }
        } else {
            m->S_inv[j] = 0.0;
            for (int i = 0; i < m->W.numRows; i++)
            	MAT_EL(m->U, i, j) = 0;
        }
    }

    for (int i = 0; i < m->num_joints; i++) {       // Righe di A+ (dimensione n)
        for (int j = 0; j < 6; j++) {   // Colonne di A+ (dimensione m)
        	float sum = 0.0;
            for (int k = 0; k < m->num_joints; k++) {
            	float v_ik = MAT_EL(m->V, i , k);
            	float u_jk = MAT_EL(m->U, j , k); // U^T[k][j] è uguale a U[j][k]
                sum += v_ik * m->S_inv[k] * u_jk;
            }
            MAT_EL(m->inv_jacobe, i , j) = sum;
        }
    }

}
