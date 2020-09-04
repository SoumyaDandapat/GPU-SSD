taskset -c 0,1 ./bin/release/vectorAdd >> VA_out
taskset -c 0,1 ./bin/release/CP >> CP_out
taskset -c 0,1 ./bin/release/MUM MUM/data/NC_003997.20k.fna MUM/data/NC_003997_q25bp.50k.fna >> MUM_out
