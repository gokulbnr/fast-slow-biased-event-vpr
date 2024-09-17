#!/usr/bin/env python

import tonic
import pandas as pd
import os
import argparse
from fastparquet import write
from tqdm import tqdm
import numpy as np
import cv2

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--traverse", type=str, default="dvs_vpr_2020-04-24-15-12-03_no_hot_pixels_nobursts_denoised.parquet")
    parser.add_argument("-l", "--event_frame_length", type=float, default=0.066)
    parser.add_argument("-mint", "--min_event_threshold", type=float, default=1.0)
    parser.add_argument("-ao", "--aps_output_directory", type=str)
    parser.add_argument("-do", "--dvs_output_directory", type=str)
    args = parser.parse_args()


    if args.dvs_output_directory is not None:
        aedat_data = tonic.io.read_davis_346(args.traverse, 'dvs')
        print("Writing DVS events to parquet file.")
        outdir = os.path.join(args.dvs_output_directory)
        os.makedirs(outdir, exist_ok=True)
        df = pd.DataFrame({'t': aedat_data[1]['t'], 'x': aedat_data[1]['x'], 'y': aedat_data[1]['y'], 'p': aedat_data[1]['p']})
        del(aedat_data)
        write(f"{outdir}/{os.path.basename(args.traverse).split('.')[0]}.parquet", df)
        print("DVS events written to parquet file.")

    if args.aps_output_directory is not None:
        aedat_data = tonic.io.read_davis_346(args.traverse, 'aps')
        print("Writing APS events to parquet file.")
        outdir = os.path.join(args.aps_output_directory, os.path.basename(args.traverse).split('.')[0])
        os.makedirs(outdir, exist_ok=True)
        for px in range(0,min(aedat_data[2].shape[0], aedat_data[3].shape[0])-(346*260), (346*260)):
            img = np.zeros((260,346), dtype=np.float64)
            img[aedat_data[2]['y'][px:px+89960], 346 - 1 -
            aedat_data[2]['x'][px:px+89960]] = aedat_data[2]['data'][px:px+89960] - aedat_data[3]['data'][px:px+89960]
            cv2.imwrite(f"{outdir}/{float(aedat_data[2]['t'][px])}.png", img)
