#! /bin/bash

exp_name=$1
exp_loc=$2
data_loc=$3
ref_set=$4

data_root_dir="${data_loc}/${exp_name}"
exp_root_dir="${exp_loc}/${exp_name}"

num_iters=$(ls ${data_root_dir} | wc -l)

for iter in $(seq 1 $num_iters); do
    mkdir -p $exp_root_dir/${iter}/descriptors
    mkdir -p $exp_root_dir/${iter}/similarity_results
    mkdir -p $exp_root_dir/${iter}/reports
    mkdir -p $exp_root_dir/${iter}/logs
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

if [ "$ref_set" = "high" ]; then
    exp_ref="h"
elif [ "$ref_set" = "low" ]; then
    exp_ref="l"
fi

# Mamba setups
eval "$(conda shell.bash hook)"
source /home/gokulbnr/mambaforge/etc/profile.d/mamba.sh
mamba activate evpr

for iter in $(seq 1 $num_iters); do
    for condition in "h" "m" "l"; do
        
        # Get Image Descriptors for all image sets
        python src/generate_image_descriptors.py \
            -I "${data_root_dir}/${iter}/dvs/frames_eventsum_moving_norm/${short_name}_${exp_ref}_ref" \
            -W "$exp_root_dir/${iter}/descriptors" \
            -M patchnorm \
            -L "$exp_root_dir/${iter}/logs";
        
        python src/generate_image_descriptors.py \
            -I "${data_root_dir}/${iter}/dvs/frames_eventsum_moving_norm/${short_name}_${condition}_qry" \
            -W "$exp_root_dir/${iter}/descriptors" \
            -M patchnorm \
            -L "$exp_root_dir/${iter}/logs";

        # Compute similarity matrix for each (reference, query) pair
        python src/generate_similarity_matrix.py \
            -R "$exp_root_dir/${iter}/descriptors/${short_name}_${exp_ref}_ref.npz" \
            -Q "$exp_root_dir/${iter}/descriptors/${short_name}_${condition}_qry.npz" \
            -W "$exp_root_dir/${iter}/similarity_results" \
            -M sad \
            -L "$exp_root_dir/${iter}/logs";

        # Get reports for all experiments
        python src/generate_reports.py \
            -I "$exp_root_dir/${iter}/similarity_results/R_${short_name}_${exp_ref}_ref_Q_${short_name}_${condition}_qry_M_sad.npz" \
            -W "$exp_root_dir/${iter}/reports" \
            -D "${data_root_dir}/${iter}/dvs/frames_eventsum_moving_norm/${short_name}_${exp_ref}_ref" \
            -E "${data_root_dir}/${iter}/dvs/frames_eventsum_moving_norm/${short_name}_${condition}_qry";

    done;
done;
