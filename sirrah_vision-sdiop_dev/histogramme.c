/* ============================================================================
 * Project   : SIRRAH VISION
 * ----------------------------------------------------------------------------
 * Title     : Histogram computation for 10-bit RAW images
 * Author    : Serigne Saliou Mbacké Diop
 * Date      : 30/09/2025
 * ----------------------------------------------------------------------------
 * Confidential file
 * Copyright (C) ARCK Sensor - All rights reserved
 * ----------------------------------------------------------------------------
 * Description :
 *    Standalone C tool for analyzing 10-bit RAW image data.
 *    Computes the histogram of pixel values and exports it to a CSV file
 *    for visualization and post-processing.
 * ----------------------------------------------------------------------------
 * History :
 *    07/11/2025  S.Diop : creation
 * ============================================================================
 */

/* ----------------------------------------------------------------------------
 * Include
 * ----------------------------------------------------------------------------
 */

/* Standard include */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

/* ----------------------------------------------------------------------------
 * Constant
 * ----------------------------------------------------------------------------
 */

#define WIDTH   1456
#define HEIGHT  1088

/* ----------------------------------------------------------------------------
 * Functions
 * ----------------------------------------------------------------------------
 */

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------------
 * Function : main
 * Description :
 *    Entry point. Reads a 10-bit RAW image, computes its histogram, and
 *    saves the results in a CSV file.
 * ----------------------------------------------------------------------------
 */
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        printf("Usage: %s image.raw\n", argv[0]);
        return 1;
    }

    const char *filename = argv[1];
    FILE *f = fopen(filename, "rb");
    if (!f)
    {
        perror(" Error opening RAW file");
        return 1;
    }

    size_t pixelCount = WIDTH * HEIGHT;
    uint16_t *buffer = malloc(pixelCount * sizeof(uint16_t));
    if (!buffer)
    {
        printf("Memory allocation error\n");
        fclose(f);
        return 1;
    }

    /* Read raw data from file */
    size_t read = fread(buffer, sizeof(uint16_t), pixelCount, f);
    fclose(f);

    if (read != pixelCount)
    {
        printf(" Incorrect file size (%zu read, expected %zu)\n", read, pixelCount);
        free(buffer);
        return 1;
    }

    /* Compute histogram (10-bit range → 0..1023) */
    unsigned long histogram[1024] = {0};

    for (size_t i = 0; i < pixelCount; i++)
    {
        /* Extract 10 most significant bits (right shift by 6 bits) */
        uint16_t pixel = buffer[i] >> 6;
        histogram[pixel]++;
    }

    free(buffer);

    printf("Histogram (10 bits) for : %s\n", filename);
    for (int i = 0; i < 1024; i++)
    {
        if (histogram[i] > 0)
            printf("%4d : %lu\n", i, histogram[i]);
    }

    /* Save results in CSV file */
    FILE *csv = fopen("histogram_10bit.csv", "w");
    if (!csv)
    {
        perror("Error creating CSV file");
        return 1;
    }

    fprintf(csv, "Value,Count\n");
    for (int i = 0; i < 1024; i++)
        fprintf(csv, "%d,%lu\n", i, histogram[i]);
    fclose(csv);

    printf("\nHistogram saved to : histogram_10bit.csv\n");
    return 0;
} /* main() */

#ifdef __cplusplus
} /* extern "C" */
#endif

/* end of file */
