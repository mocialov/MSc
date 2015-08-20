//reads points.dat file
//where 1st line: num_data_points, num_clusters, num_dimensions
//2nd line fuzziness, epsilon
//for num_data_points (300) and for num_dimensions (2)
//		
//not consistent clusters!

#define MAX_DATA_POINTS 10000
#define MAX_CLUSTER 100
#define MAX_DATA_DIMENSION 5
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "fcm.h"

int num_data_points;
int num_clusters;
int num_dimensions;
float low_high[MAX_DATA_DIMENSION][2];
float degree_of_memb[MAX_DATA_POINTS][MAX_CLUSTER];
float epsilon;
float fuzziness;
float data_point[MAX_DATA_POINTS][MAX_DATA_DIMENSION];
float cluster_centre[MAX_CLUSTER][MAX_DATA_DIMENSION];

Point createPoint (int tag, float newX, float newY, float newZ){
    Point newPoint;

    newPoint.tag = tag;
    newPoint.x = newX;
    newPoint.y = newY;
    newPoint.z = newZ;

    return newPoint;
}

int
init(int reverse_flag, int num_data_points_par, int num_clusters_par, int num_dimensions_par, float fuzziness_par, float epsilon_par, float data_point_par[num_data_points_par][num_dimensions_par]) {
    int i, j, r, rval;
    float s;
    
    num_data_points = num_data_points_par;
    num_clusters = num_clusters_par;
    num_dimensions = num_dimensions_par;
    fuzziness = fuzziness_par;
    epsilon = epsilon_par;
    
    if (num_clusters > MAX_CLUSTER) {
        printf("Number of clusters should be < %d\n", MAX_CLUSTER);
        goto failure;
    }
    
    if (num_data_points > MAX_DATA_POINTS) {
        printf("Number of data points should be < %d\n", MAX_DATA_POINTS);
        goto failure;
    }
    
    if (num_dimensions > MAX_DATA_DIMENSION) {
        printf("Number of dimensions should be >= 1.0 and < %d\n",
                MAX_DATA_DIMENSION);
        goto failure;
    }
    
    if (fuzziness <= 1.0) {
        printf("Fuzzyness coefficient should be > 1.0\n");
        goto failure;
    }
    
    if (epsilon <= 0.0 || epsilon > 1.0) {
        printf("Termination criterion should be > 0.0 and <= 1.0\n");
        goto failure;
    }

    for (i = 0; i < num_data_points; i++) {
        for (j = 0; j < num_dimensions; j++) {
            data_point[reverse_flag ? j : i][reverse_flag ? i : j] = data_point_par[i][j];
            if (data_point[reverse_flag ? j : i][reverse_flag ? i : j] < low_high[j][0])
                low_high[j][0] = data_point[reverse_flag ? j : i][reverse_flag ? i : j];
            if (data_point[reverse_flag ? j : i][reverse_flag ? i : j] > low_high[j][1])
                low_high[j][1] = data_point[reverse_flag ? j : i][reverse_flag ? i : j];
        }
    }
    for (i = 0; i < num_data_points; i++) {
        s = 0.0;
        r = 100;
        for (j = 1; j < num_clusters; j++) {
            rval = rand() % (r + 1);
            r -= rval;
            degree_of_memb[i][j] = rval / 100.0;
            s += degree_of_memb[i][j];
        }
        degree_of_memb[i][0] = 1.0 - s;
    }

    return 0;

failure:
    exit(1);
}

int
calculate_centre_vectors() {
    int i, j, k;
    float numerator, denominator;
    float t[MAX_DATA_POINTS][MAX_CLUSTER];
    for (i = 0; i < num_data_points; i++) {
        for (j = 0; j < num_clusters; j++) {
            t[i][j] = pow(degree_of_memb[i][j], fuzziness);
        }
    }
    for (j = 0; j < num_clusters; j++) {
        for (k = 0; k < num_dimensions; k++) {
            numerator = 0.0;
            denominator = 0.0;
            for (i = 0; i < num_data_points; i++) {
                numerator += t[i][j] * data_point[i][k];
                denominator += t[i][j];
            }
            cluster_centre[j][k] = numerator / denominator;
        }
    }
    return 0;
}

float
get_norm(int i, int j) {
    int k;
    float sum = 0.0;
    for (k = 0; k < num_dimensions; k++) {
        sum += pow(data_point[i][k] - cluster_centre[j][k], 2);
    }
    return sqrt(sum);
}

float
get_new_value(int i, int j) {
    int k;
    float t, p, sum;
    sum = 0.0;
    p = 2 / (fuzziness - 1);
    for (k = 0; k < num_clusters; k++) {
        t = get_norm(i, j) / get_norm(i, k);
        t = pow(t, p);
        sum += t;
    }
    return 1.0 / sum;
}

float
update_degree_of_membership() {
    int i, j;
    float new_uij;
    float max_diff = 0.0, diff;
    for (j = 0; j < num_clusters; j++) {
        for (i = 0; i < num_data_points; i++) {
            new_uij = get_new_value(i, j);
            diff = new_uij - degree_of_memb[i][j];
            if (diff > max_diff)
                max_diff = diff;
            degree_of_memb[i][j] = new_uij;
        }
    }
    return max_diff;
}

int
fcm(int reverse_flag, int num_data_points_par, int num_clusters_par, int num_dimensions_par, float fuzziness_par, float epsilon_par, float data_point_par[num_data_points_par][num_dimensions_par]) {
    float max_diff;
    init(reverse_flag, num_data_points_par, num_clusters_par, num_dimensions_par, fuzziness_par, epsilon_par, data_point_par);
    do {
        calculate_centre_vectors();
        max_diff = update_degree_of_membership();
    } while (max_diff > epsilon);
    return 0;
}

int
gnuplot_membership_matrix() {
    int i, j, cluster;
    char fname[100];
    float highest;
    FILE * f[MAX_CLUSTER];
    if (num_dimensions != 2) {
        printf("Plotting the cluster only works when the\n");
        printf("number of dimensions is two. This will create\n");
        printf("a two-dimensional plot of the cluster points.\n");
        exit(1);
    }
    for (j = 0; j < num_clusters; j++) {
        sprintf(fname, "cluster.%d", j);
        if ((f[j] = fopen(fname, "w")) == NULL) {
            printf("Could not create %s\n", fname);
            for (i = 0; i < j; i++) {
                fclose(f[i]);
                sprintf(fname, "cluster.%d", i);
                remove(fname);
            }
            return -1;
        }
        fprintf(f[j], "#Data points for cluster: %d\n", j);
    }
    for (i = 0; i < num_data_points; i++) {
        cluster = 0;
        highest = 0.0;
        for (j = 0; j < num_clusters; j++) {
            if (degree_of_memb[i][j] > highest) {
                highest = degree_of_memb[i][j];
                cluster = j;
            }
        }
        fprintf(f[cluster], "%lf %lf\n", data_point[i][0], data_point[i][1]);
    }
    for (j = 0; j < num_clusters; j++) {
        fclose(f[j]);
    }
    if ((f[0] = fopen("gnuplot.script", "w")) == NULL) {
        printf("Could not create gnuplot.script.\n");
        for (i = 0; i < j; i++) {
            fclose(f[i]);
            sprintf(fname, "cluster.%d", i);
            remove(fname);
        }
        return -1;
    }
    fprintf(f[0], "set terminal png medium\n");
    fprintf(f[0], "set output \"cluster_plot.png\"\n");
    fprintf(f[0], "set title \"FCM clustering\"\n");
    fprintf(f[0], "set xlabel \"x-coordinate\"\n");
    fprintf(f[0], "set ylabel \"y-coordinate\"\n");
    fprintf(f[0], "set xrange [%lf : %lf]\n", low_high[0][0], low_high[0][1]);
    fprintf(f[0], "set yrange [%lf : %lf]\n", low_high[1][0], low_high[1][1]);
    fprintf(f[0],
            "plot 'cluster.0' using 1:2 with points pt 7 ps 1 lc 1 notitle");
    for (j = 1; j < num_clusters; j++) {
        sprintf(fname, "cluster.%d", j);
        fprintf(f[0],
                ",\\\n'%s' using 1:2 with points  pt 7 ps 1 lc %d notitle",
                fname, j + 1);
    }
    fprintf(f[0], "\n");
    fclose(f[0]);
    return 0;
}

void
print_data_points(char *fname) {
    int i, j;
    FILE *f;
    if (fname == NULL)
        f = stdout;
    else if ((f = fopen(fname, "w")) == NULL) {
        printf("Cannot create output file.\n");
        exit(1);
    }
    fprintf(f, "Data points:\n");
    for (i = 0; i < num_data_points; i++) {
        printf("Data[%d]: ", i);
        for (j = 0; j < num_dimensions; j++) {
            printf("%.5lf ", data_point[i][j]);
        }
        printf("\n");
    }
    if (fname == NULL)
        fclose(f);
}

void
print_membership_matrix(char *fname) {
    int i, j;
    FILE *f;
    if (fname == NULL)
        f = stdout;
    else if ((f = fopen(fname, "w")) == NULL) {
        printf("Cannot create output file.\n");
        exit(1);
    }
    fprintf(f, "Membership matrix:\n");
    for (i = 0; i < num_data_points; i++) {
        fprintf(f, "Data[%d]: ", i);
        for (j = 0; j < num_clusters; j++) {
            fprintf(f, "%lf ", degree_of_memb[i][j]);
        }
        fprintf(f, "\n");
    }
    if (fname == NULL)
        fclose(f);
}

int
fcm_run(int reverse_flag, int print_flag, float threshold, int num_data_points_par, int num_clusters_par, int num_dimensions_par, float fuzziness_par, float epsilon_par, float data_point_par[num_data_points_par][2], float membership_matrix[num_clusters_par][2][num_data_points_par]) {            
    //int i;
    //float data_point_par_expanded[num_data_points_par][2];
    
   //for (i=0; i<num_data_points_par;i++){
   //  data_point_par_expanded[i][0] = ((Point)data_point_par[i]).x;
   //  data_point_par_expanded[i][1] = ((Point)data_point_par[i]).y;
   //}
   
   int findMaxY(data_point_par[num_data_points_par][2]);
   
   
   fcm(reverse_flag, num_data_points_par, num_clusters_par, num_dimensions_par, fuzziness_par, epsilon_par, data_point_par);

    if(print_flag){
     print_membership_matrix("membership.matrix");
     gnuplot_membership_matrix();
    }


    //float data[num_clusters][num_data_points];
    memset(membership_matrix, 0, num_clusters_par * 2 * num_data_points_par * sizeof(float));
    
    int i, j, m, cluster;
    float highest;

    for (i = 0; i < num_data_points; i++) {
        cluster = 0;
        highest = 0.0;
        for (j = 0; j < num_clusters; j++) {
            if (degree_of_memb[i][j] > highest) {
                highest = degree_of_memb[i][j];
                cluster = j;
            }
        }
        
        membership_matrix[cluster][0][i] = data_point[i][0];//createPoint(-1, data_point[i][0], data_point[i][1], 0.0);
        membership_matrix[cluster][1][i] = data_point[i][1];
    }
    

memset(low_high, 0, sizeof(low_high));
memset(degree_of_memb, 0, sizeof(degree_of_memb));
memset(data_point, 0, sizeof(data_point));
memset(cluster_centre, 0, sizeof(cluster_centre));    
    
    
   return 0;
}
