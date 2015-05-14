std_msgs/Header         header          # header for time / frame information
sensor_msgs/LaserScan   scan            # the processed scan
int32[]                 mask            # cluster id's of the rays
float32[]               linearity       # linearity score as in LOAM paper
float32[9]              fim             # Fisher Information Matrix
geometry_msgs/Point[]   points_3d       # transformed points (if any)
int32[]                 line_idxs       # every consecutive two index, defines
                                        # the two ends of a lines. Indexes point
                                        # to rays as in the scan.ranges vector.
geometry_msgs/Point[]   points_2d       # euclidean correspondences of the rays.


