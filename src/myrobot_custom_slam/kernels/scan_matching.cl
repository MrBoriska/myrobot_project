typedef struct {
    float x;
    float y;
} Point2D;

// Include gradients in output for IESKF Jacobian calculation
__kernel void scan_match(__global const Point2D* scan, __global float* errors, __global float* gradients_x, __global float* gradients_y, __global const float* pose, __global const Point2D* map, int map_size) {
    int id = get_global_id(0);

    // pose: x, y, yaw
    float px = pose[0];
    float py = pose[1];
    float yaw = pose[2];

    float cos_yaw = cos(yaw);
    float sin_yaw = sin(yaw);

    // Transform point from local to global (map) frame
    float lx = scan[id].x;
    float ly = scan[id].y;

    float gx = px + lx * cos_yaw - ly * sin_yaw;
    float gy = py + lx * sin_yaw + ly * cos_yaw;

    // Nearest neighbor search in map (brute force for small local maps, grid based normally)
    float min_dist2 = 1e9f;
    float best_dx = 0.0f;
    float best_dy = 0.0f;

    for(int i = 0; i < map_size; ++i) {
        float dx = gx - map[i].x;
        float dy = gy - map[i].y;
        float dist2 = dx*dx + dy*dy;
        if(dist2 < min_dist2) {
            min_dist2 = dist2;
            best_dx = dx;
            best_dy = dy;
        }
    }

    // Distance error
    errors[id] = sqrt(min_dist2);

    // Gradients (direction of error)
    if (min_dist2 > 1e-6f) {
        gradients_x[id] = best_dx / sqrt(min_dist2);
        gradients_y[id] = best_dy / sqrt(min_dist2);
    } else {
        gradients_x[id] = 0.0f;
        gradients_y[id] = 0.0f;
    }
}
