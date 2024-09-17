#! /bin/bash

exp_name=$1
num_iters=$2
home_loc=$3
write_loc=$4

data_root_dir="${home_loc}/${exp_name}"
data_write_dir="${write_loc}/${exp_name}"

for iter in $(seq 1 $num_iters); do
    mkdir -p $data_write_dir/${iter}/aps/frames
    mkdir -p $data_write_dir/${iter}/dvs/parquets
    mkdir -p $data_write_dir/${iter}/odom
done;

if [ "$exp_name" = "default_params" ]; then
    short_name="df"
elif [ "$exp_name" = "PxBw" ]; then
    short_name="ln"
elif [ "$exp_name" = "RfPr" ]; then
    short_name="lr"
elif [ "$exp_name" = "PxTh" ]; then
    short_name="th"
elif [ "$exp_name" = "Fast_Slow" ]; then
    short_name="c"
elif [ "$exp_name" = "constant" ]; then
    short_name="ab"
elif [ "$exp_name" = "fast" ]; then
    short_name="ab"
elif [ "$exp_name" = "slow" ]; then
    short_name="ab"
elif [ "$exp_name" = "n2" ]; then
    short_name="ab"
elif [ "$exp_name" = "n7" ]; then
    short_name="ab"
elif [ "$exp_name" = "n10" ]; then
    short_name="ab"
fi
echo "Experiment Name:  ${exp_name}"
echo "Short Name:  ${short_name}"

# Mamba setups
eval "$(conda shell.bash hook)"
source /home/gokulbnr/mambaforge/etc/profile.d/mamba.sh
mamba activate evpr

for iter in $(seq 1 $num_iters); do
    echo $iter
    for condition in "h" "m" "l"; do
        for traverse in "ref" "qry"; do
            
            # Raw Data Processing - Events
            python src/aedat_to_parquets.py \
                -t "$data_root_dir/${iter}/aedats/${short_name}_${condition}_${traverse}.aedat" \
                -ao "$data_write_dir/${iter}/aps/frames/" \
                -do "$data_write_dir/${iter}/dvs/parquets/"

            # # Mamba setups
            # # mamba deactivate
            # mamba activate ros

            # Raw Data Processing - Ground Truth Robot Poses
            python src/rosbag_to_parquets.py \
                -r "$data_root_dir/${iter}/bags/${short_name}_${condition}_${traverse}.bag" \
                -o "$data_write_dir/${iter}/odom/"

            # mamba activate general-env

            # Generating Event Frames
            python src/event_frames_with_ground_truth.py \
                -t "$data_write_dir/${iter}/dvs/parquets/${short_name}_${condition}_${traverse}.parquet" \
                -out_dir "$data_write_dir/${iter}/dvs/frames_eventsum/" \
                -gt "$data_write_dir/${iter}/odom/${short_name}_${condition}_${traverse}.parquet" \
                -mint 1 \
                -maxt 100 \
                -l 0.066

            echo "SEE"
            echo "$data_write_dir/${iter}/dvs/parquets/${short_name}_${condition}_${traverse}.parquet"
            echo "$data_write_dir/${iter}/odom/${short_name}_${condition}_${traverse}.parquet"

            # Post-Processng
            python src/remove_stationary_frames_without_interpolation.py \
                -I "$data_write_dir/${iter}/dvs/frames_eventsum/${short_name}_${condition}_${traverse}" \
                -W "$data_write_dir/${iter}/dvs/frames_eventsum_moving/${short_name}_${condition}_${traverse}"

            python src/normalize_frames.py \
                -I "$data_write_dir/${iter}/dvs/frames_eventsum_moving/${short_name}_${condition}_${traverse}" \
                -W "$data_write_dir/${iter}/dvs/frames_eventsum_moving_norm/${short_name}_${condition}_${traverse}"
        done;
    done;
done;
