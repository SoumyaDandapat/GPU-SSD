taskset -c 6,7 ./bin/release/RAY 1024 1024 >> RAY_out
taskset -c 6,7 ./bin/release/WP << 10 ./data/ >> WP_out
