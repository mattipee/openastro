/*****************************************************************************
 *
 * yuvlut.h -- YUV lookup tables
 *
 * Copyright 2014 James Fidell (james@openastroproject.org)
 *
 * License:
 *
 * This file is part of the Open Astro Project.
 *
 * The Open Astro Project is free software: you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as published
 * by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The Open Astro Project is distributed in the hope that it will be
 * useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the Open Astro Project.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 *****************************************************************************/

#ifndef OPENASTRO_VIDEO_YUVLUT_H
#define OPENASTRO_VIDEO_YUVLUT_H

static float lut_1_4075[256] = {
    -180.16, -178.75, -177.34, -175.94, -174.53, -173.12, -171.72, -170.31,
    -168.90, -167.49, -166.09, -164.68, -163.27, -161.86, -160.45, -159.05,
    -157.64, -156.23, -154.82, -153.42, -152.01, -150.60, -149.19, -147.79,
    -146.38, -144.97, -143.56, -142.16, -140.75, -139.34, -137.94, -136.53,
    -135.12, -133.71, -132.31, -130.90, -129.49, -128.08, -126.67, -125.27,
    -123.86, -122.45, -121.05, -119.64, -118.23, -116.82, -115.41, -114.01,
    -112.60, -111.19, -109.78, -108.38, -106.97, -105.56, -104.16, -102.75,
    -101.34, -99.93, -98.52, -97.12, -95.71, -94.30, -92.89, -91.49,
    -90.08, -88.67, -87.27, -85.86, -84.45, -83.04, -81.64, -80.23,
    -78.82, -77.41, -76.00, -74.60, -73.19, -71.78, -70.38, -68.97,
    -67.56, -66.15, -64.75, -63.34, -61.93, -60.52, -59.12, -57.71,
    -56.30, -54.89, -53.48, -52.08, -50.67, -49.26, -47.85, -46.45,
    -45.04, -43.63, -42.23, -40.82, -39.41, -38.00, -36.59, -35.19,
    -33.78, -32.37, -30.96, -29.56, -28.15, -26.74, -25.34, -23.93,
    -22.52, -21.11, -19.70, -18.30, -16.89, -15.48, -14.07, -12.67,
    -11.26, -9.85, -8.45, -7.04, -5.63, -4.22, -2.81, -1.41,
    0.00, 1.41, 2.81, 4.22, 5.63, 7.04, 8.45, 9.85,
    11.26, 12.67, 14.07, 15.48, 16.89, 18.30, 19.70, 21.11,
    22.52, 23.93, 25.34, 26.74, 28.15, 29.56, 30.96, 32.37,
    33.78, 35.19, 36.59, 38.00, 39.41, 40.82, 42.23, 43.63,
    45.04, 46.45, 47.85, 49.26, 50.67, 52.08, 53.48, 54.89,
    56.30, 57.71, 59.12, 60.52, 61.93, 63.34, 64.75, 66.15,
    67.56, 68.97, 70.38, 71.78, 73.19, 74.60, 76.00, 77.41,
    78.82, 80.23, 81.64, 83.04, 84.45, 85.86, 87.27, 88.67,
    90.08, 91.49, 92.89, 94.30, 95.71, 97.12, 98.52, 99.93,
    101.34, 102.75, 104.16, 105.56, 106.97, 108.38, 109.78, 111.19,
    112.60, 114.01, 115.41, 116.82, 118.23, 119.64, 121.05, 122.45,
    123.86, 125.27, 126.67, 128.08, 129.49, 130.90, 132.31, 133.71,
    135.12, 136.53, 137.94, 139.34, 140.75, 142.16, 143.56, 144.97,
    146.38, 147.79, 149.19, 150.60, 152.01, 153.42, 154.82, 156.23,
    157.64, 159.05, 160.45, 161.86, 163.27, 164.68, 166.09, 167.49,
    168.90, 170.31, 171.72, 173.12, 174.53, 175.94, 177.34, 178.75
};

static float lut_0_3455[256] = {
    -44.22, -43.88, -43.53, -43.19, -42.84, -42.50, -42.15, -41.81,
    -41.46, -41.11, -40.77, -40.42, -40.08, -39.73, -39.39, -39.04,
    -38.70, -38.35, -38.00, -37.66, -37.31, -36.97, -36.62, -36.28,
    -35.93, -35.59, -35.24, -34.90, -34.55, -34.20, -33.86, -33.51,
    -33.17, -32.82, -32.48, -32.13, -31.79, -31.44, -31.09, -30.75,
    -30.40, -30.06, -29.71, -29.37, -29.02, -28.68, -28.33, -27.99,
    -27.64, -27.29, -26.95, -26.60, -26.26, -25.91, -25.57, -25.22,
    -24.88, -24.53, -24.18, -23.84, -23.49, -23.15, -22.80, -22.46,
    -22.11, -21.77, -21.42, -21.08, -20.73, -20.38, -20.04, -19.69,
    -19.35, -19.00, -18.66, -18.31, -17.97, -17.62, -17.27, -16.93,
    -16.58, -16.24, -15.89, -15.55, -15.20, -14.86, -14.51, -14.17,
    -13.82, -13.47, -13.13, -12.78, -12.44, -12.09, -11.75, -11.40,
    -11.06, -10.71, -10.36, -10.02, -9.67, -9.33, -8.98, -8.64,
    -8.29, -7.95, -7.60, -7.26, -6.91, -6.56, -6.22, -5.87,
    -5.53, -5.18, -4.84, -4.49, -4.15, -3.80, -3.45, -3.11,
    -2.76, -2.42, -2.07, -1.73, -1.38, -1.04, -0.69, -0.35,
    0.00, 0.35, 0.69, 1.04, 1.38, 1.73, 2.07, 2.42,
    2.76, 3.11, 3.45, 3.80, 4.15, 4.49, 4.84, 5.18,
    5.53, 5.87, 6.22, 6.56, 6.91, 7.26, 7.60, 7.95,
    8.29, 8.64, 8.98, 9.33, 9.67, 10.02, 10.36, 10.71,
    11.06, 11.40, 11.75, 12.09, 12.44, 12.78, 13.13, 13.47,
    13.82, 14.17, 14.51, 14.86, 15.20, 15.55, 15.89, 16.24,
    16.58, 16.93, 17.27, 17.62, 17.97, 18.31, 18.66, 19.00,
    19.35, 19.69, 20.04, 20.38, 20.73, 21.08, 21.42, 21.77,
    22.11, 22.46, 22.80, 23.15, 23.49, 23.84, 24.18, 24.53,
    24.88, 25.22, 25.57, 25.91, 26.26, 26.60, 26.95, 27.29,
    27.64, 27.99, 28.33, 28.68, 29.02, 29.37, 29.71, 30.06,
    30.40, 30.75, 31.09, 31.44, 31.79, 32.13, 32.48, 32.82,
    33.17, 33.51, 33.86, 34.20, 34.55, 34.90, 35.24, 35.59,
    35.93, 36.28, 36.62, 36.97, 37.31, 37.66, 38.00, 38.35,
    38.70, 39.04, 39.39, 39.73, 40.08, 40.42, 40.77, 41.11,
    41.46, 41.81, 42.15, 42.50, 42.84, 43.19, 43.53, 43.88
};

static float lut_0_7169[256] = {
    -91.76, -91.05, -90.33, -89.61, -88.90, -88.18, -87.46, -86.74,
    -86.03, -85.31, -84.59, -83.88, -83.16, -82.44, -81.73, -81.01,
    -80.29, -79.58, -78.86, -78.14, -77.43, -76.71, -75.99, -75.27,
    -74.56, -73.84, -73.12, -72.41, -71.69, -70.97, -70.26, -69.54,
    -68.82, -68.11, -67.39, -66.67, -65.95, -65.24, -64.52, -63.80,
    -63.09, -62.37, -61.65, -60.94, -60.22, -59.50, -58.79, -58.07,
    -57.35, -56.64, -55.92, -55.20, -54.48, -53.77, -53.05, -52.33,
    -51.62, -50.90, -50.18, -49.47, -48.75, -48.03, -47.32, -46.60,
    -45.88, -45.16, -44.45, -43.73, -43.01, -42.30, -41.58, -40.86,
    -40.15, -39.43, -38.71, -38.00, -37.28, -36.56, -35.84, -35.13,
    -34.41, -33.69, -32.98, -32.26, -31.54, -30.83, -30.11, -29.39,
    -28.68, -27.96, -27.24, -26.53, -25.81, -25.09, -24.37, -23.66,
    -22.94, -22.22, -21.51, -20.79, -20.07, -19.36, -18.64, -17.92,
    -17.21, -16.49, -15.77, -15.05, -14.34, -13.62, -12.90, -12.19,
    -11.47, -10.75, -10.04, -9.32, -8.60, -7.89, -7.17, -6.45,
    -5.74, -5.02, -4.30, -3.58, -2.87, -2.15, -1.43, -0.72,
    0.00, 0.72, 1.43, 2.15, 2.87, 3.58, 4.30, 5.02,
    5.74, 6.45, 7.17, 7.89, 8.60, 9.32, 10.04, 10.75,
    11.47, 12.19, 12.90, 13.62, 14.34, 15.05, 15.77, 16.49,
    17.21, 17.92, 18.64, 19.36, 20.07, 20.79, 21.51, 22.22,
    22.94, 23.66, 24.37, 25.09, 25.81, 26.53, 27.24, 27.96,
    28.68, 29.39, 30.11, 30.83, 31.54, 32.26, 32.98, 33.69,
    34.41, 35.13, 35.84, 36.56, 37.28, 38.00, 38.71, 39.43,
    40.15, 40.86, 41.58, 42.30, 43.01, 43.73, 44.45, 45.16,
    45.88, 46.60, 47.32, 48.03, 48.75, 49.47, 50.18, 50.90,
    51.62, 52.33, 53.05, 53.77, 54.48, 55.20, 55.92, 56.64,
    57.35, 58.07, 58.79, 59.50, 60.22, 60.94, 61.65, 62.37,
    63.09, 63.80, 64.52, 65.24, 65.95, 66.67, 67.39, 68.11,
    68.82, 69.54, 70.26, 70.97, 71.69, 72.41, 73.12, 73.84,
    74.56, 75.27, 75.99, 76.71, 77.43, 78.14, 78.86, 79.58,
    80.29, 81.01, 81.73, 82.44, 83.16, 83.88, 84.59, 85.31,
    86.03, 86.74, 87.46, 88.18, 88.90, 89.61, 90.33, 91.05
};

static float lut_1_7790[256] = {
    -227.71, -225.93, -224.15, -222.38, -220.60, -218.82, -217.04, -215.26,
    -213.48, -211.70, -209.92, -208.14, -206.36, -204.58, -202.81, -201.03,
    -199.25, -197.47, -195.69, -193.91, -192.13, -190.35, -188.57, -186.79,
    -185.02, -183.24, -181.46, -179.68, -177.90, -176.12, -174.34, -172.56,
    -170.78, -169.00, -167.23, -165.45, -163.67, -161.89, -160.11, -158.33,
    -156.55, -154.77, -152.99, -151.22, -149.44, -147.66, -145.88, -144.10,
    -142.32, -140.54, -138.76, -136.98, -135.20, -133.42, -131.65, -129.87,
    -128.09, -126.31, -124.53, -122.75, -120.97, -119.19, -117.41, -115.63,
    -113.86, -112.08, -110.30, -108.52, -106.74, -104.96, -103.18, -101.40,
    -99.62, -97.84, -96.07, -94.29, -92.51, -90.73, -88.95, -87.17,
    -85.39, -83.61, -81.83, -80.05, -78.28, -76.50, -74.72, -72.94,
    -71.16, -69.38, -67.60, -65.82, -64.04, -62.27, -60.49, -58.71,
    -56.93, -55.15, -53.37, -51.59, -49.81, -48.03, -46.25, -44.47,
    -42.70, -40.92, -39.14, -37.36, -35.58, -33.80, -32.02, -30.24,
    -28.46, -26.68, -24.91, -23.13, -21.35, -19.57, -17.79, -16.01,
    -14.23, -12.45, -10.67, -8.89, -7.12, -5.34, -3.56, -1.78,
    0.00, 1.78, 3.56, 5.34, 7.12, 8.89, 10.67, 12.45,
    14.23, 16.01, 17.79, 19.57, 21.35, 23.13, 24.91, 26.68,
    28.46, 30.24, 32.02, 33.80, 35.58, 37.36, 39.14, 40.92,
    42.70, 44.47, 46.25, 48.03, 49.81, 51.59, 53.37, 55.15,
    56.93, 58.71, 60.49, 62.27, 64.04, 65.82, 67.60, 69.38,
    71.16, 72.94, 74.72, 76.50, 78.28, 80.05, 81.83, 83.61,
    85.39, 87.17, 88.95, 90.73, 92.51, 94.29, 96.07, 97.84,
    99.62, 101.40, 103.18, 104.96, 106.74, 108.52, 110.30, 112.08,
    113.86, 115.63, 117.41, 119.19, 120.97, 122.75, 124.53, 126.31,
    128.09, 129.87, 131.65, 133.42, 135.20, 136.98, 138.76, 140.54,
    142.32, 144.10, 145.88, 147.66, 149.44, 151.22, 152.99, 154.77,
    156.55, 158.33, 160.11, 161.89, 163.67, 165.45, 167.23, 169.00,
    170.78, 172.56, 174.34, 176.12, 177.90, 179.68, 181.46, 183.24,
    185.02, 186.79, 188.57, 190.35, 192.13, 193.91, 195.69, 197.47,
    199.25, 201.03, 202.81, 204.58, 206.36, 208.14, 209.92, 211.70,
    213.48, 215.26, 217.04, 218.82, 220.60, 222.38, 224.15, 225.93
};

#endif /* OPENASTRO_VIDEO_YUVLUT_H */
