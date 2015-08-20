typedef struct
{
    int tag;
    float x;
    float y;
    float z;
} Point;

Point createPoint (int tag, float newX, float newY, float newZ);

int fcm_run(int reverse_flag, int print_flag, float threshold, int num_data_points_par, int num_clusters_par, int num_dimensions_par, float fuzziness_par, float epsilon_par, float data_point_par[num_data_points_par][2], float membership_matrix[num_clusters_par][2][num_data_points_par]);