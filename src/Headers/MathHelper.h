/**
 * @file MathHelper.h
 * @author Diogo Sousa
 * @brief Math helpers for computing homographys, based on intel realsense library examples (this header may be in future useless!)
 * @version 1.0
 * @date 2021-08-09
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef MATHHELPER_H
#define MATHHELPER_H

#include <math.h>
#include <librealsense2/rsutil.h>
#include <apriltag/apriltag.h>




static void deproject(double point[2], const rs2_intrinsics &intrinsics, const double px[2])
{
    float fpt[3], fpx[2] = {(float)px[0], (float)px[1]};
    rs2_deproject_pixel_to_point(fpt, &intrinsics, fpx, 1.0f);
    point[0] = fpt[0];
    point[1] = fpt[1];
}


static void homography_compute2(const double c[4][4], matd_t *H)
{
    double A[] = {
        c[0][0],
        c[0][1],
        1,
        0,
        0,
        0,
        -c[0][0] * c[0][2],
        -c[0][1] * c[0][2],
        c[0][2],
        0,
        0,
        0,
        c[0][0],
        c[0][1],
        1,
        -c[0][0] * c[0][3],
        -c[0][1] * c[0][3],
        c[0][3],
        c[1][0],
        c[1][1],
        1,
        0,
        0,
        0,
        -c[1][0] * c[1][2],
        -c[1][1] * c[1][2],
        c[1][2],
        0,
        0,
        0,
        c[1][0],
        c[1][1],
        1,
        -c[1][0] * c[1][3],
        -c[1][1] * c[1][3],
        c[1][3],
        c[2][0],
        c[2][1],
        1,
        0,
        0,
        0,
        -c[2][0] * c[2][2],
        -c[2][1] * c[2][2],
        c[2][2],
        0,
        0,
        0,
        c[2][0],
        c[2][1],
        1,
        -c[2][0] * c[2][3],
        -c[2][1] * c[2][3],
        c[2][3],
        c[3][0],
        c[3][1],
        1,
        0,
        0,
        0,
        -c[3][0] * c[3][2],
        -c[3][1] * c[3][2],
        c[3][2],
        0,
        0,
        0,
        c[3][0],
        c[3][1],
        1,
        -c[3][0] * c[3][3],
        -c[3][1] * c[3][3],
        c[3][3],
    };

    double epsilon = 1e-10;

    // Eliminate.
    for (int col = 0; col < 8; col++)
    {
        // Find best row to swap with.
        double max_val = 0;
        int max_val_idx = -1;
        for (int row = col; row < 8; row++)
        {
            double val = fabs(A[row * 9 + col]);
            if (val > max_val)
            {
                max_val = val;
                max_val_idx = row;
            }
        }

        if (max_val < epsilon)
        {
            fprintf(stderr, "WRN: Matrix is singular.\n");
        }

        // Swap to get best row.
        if (max_val_idx != col)
        {
            for (int i = col; i < 9; i++)
            {
                double tmp = A[col * 9 + i];
                A[col * 9 + i] = A[max_val_idx * 9 + i];
                A[max_val_idx * 9 + i] = tmp;
            }
        }

        // Do eliminate.
        for (int i = col + 1; i < 8; i++)
        {
            double f = A[i * 9 + col] / A[col * 9 + col];
            A[i * 9 + col] = 0;
            for (int j = col + 1; j < 9; j++)
            {
                A[i * 9 + j] -= f * A[col * 9 + j];
            }
        }
    }

    // Back solve.
    for (int col = 7; col >= 0; col--)
    {
        double sum = 0;
        for (int i = col + 1; i < 8; i++)
        {
            sum += A[col * 9 + i] * A[i * 9 + 8];
        }
        A[col * 9 + 8] = (A[col * 9 + 8] - sum) / A[col * 9 + col];
    }
    H->data[0] = A[8];
    H->data[1] = A[17];
    H->data[2] = A[26];
    H->data[3] = A[35];
    H->data[4] = A[44];
    H->data[5] = A[53];
    H->data[6] = A[62];
    H->data[7] = A[71];
    H->data[8] = 1;
}


static void undistort(apriltag_detection_t &detection, const rs2_intrinsics &intrinsics)
{
    deproject(detection.c, intrinsics, detection.c);
    double corr_arr[4][4];
    for (int corner = 0; corner < 4; corner++)
    {
        deproject(detection.p[corner], intrinsics, detection.p[corner]);
        
        corr_arr[corner][0] = (corner == 0 || corner == 3) ? -1 : 1; // tag corners in an ideal image
        corr_arr[corner][1] = (corner == 0 || corner == 1) ? -1 : 1; // tag corners in an ideal image
        corr_arr[corner][2] = detection.p[corner][0];           // tag corners in undistorted image focal length = 1
        corr_arr[corner][3] = detection.p[corner][1];           // tag corners in undistorted image focal length = 1
    }

    if (detection.H == nullptr)
    {
        detection.H = matd_create(3, 3);
    }
    homography_compute2(corr_arr, detection.H);
}



#endif