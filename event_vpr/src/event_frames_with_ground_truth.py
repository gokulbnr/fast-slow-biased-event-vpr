import argparse
import os
import pandas as pd
import tonic
import numpy as np
import cv2
from tqdm.auto import tqdm
from tqdm.contrib import tzip

def load_event_streams(event_streams_to_load):
    event_streams = []
    for event_stream in event_streams_to_load:
        event_streams.append(pd.read_parquet(event_stream))
    return event_streams

def print_duration(event_stream):
    print(f'Duration: {((event_stream.iloc[-1]["t"] - event_stream.iloc[0]["t"]) / 1e6):.2f}s (which is {len(event_stream)} events)')

def get_size(event_stream):
    im_width, im_height = int(event_stream["x"].max() + 1), int(event_stream["y"].max() + 1)
    return im_width, im_height

def get_times_for_streams_const_time(event_stream, time_window, overlap, include_incomplete=False):
    times = event_stream["t"]
    stride = time_window - overlap

    last_time = times.iloc[-1] if isinstance(times, pd.Series) else times[-1]
    begin_time = times.iloc[0] if isinstance(times, pd.Series) else times[0]

    if include_incomplete:
        n_slices = int(np.ceil(((last_time - begin_time) - time_window) / stride) + 1)
    else:
        n_slices = int(np.floor(((last_time - begin_time) - time_window) / stride) + 1)

    window_start_times = np.arange(n_slices) * stride
    # window_end_times = window_start_times + time_window

    return window_start_times

def get_percentage_pixels_with_events(event_frame, num_total_pixels=None):
    if num_total_pixels is None:
        num_total_pixels = event_frame.shape[0] * event_frame.shape[1]
    return np.count_nonzero(event_frame) / num_total_pixels * 100

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-t", "--traverse", type=str, default="dvs_vpr_2020-04-24-15-12-03_no_hot_pixels_nobursts_denoised.parquet")
    parser.add_argument("-gt", "--groundtruth_filepath", type=str, default="~/Data/")
    parser.add_argument("-l", "--event_frame_length", type=float, default=0.066)
    parser.add_argument("-mint", "--min_event_threshold", type=float, default=1.0)
    parser.add_argument("-maxt", "--max_event_threshold", type=float, default=90.0)
    parser.add_argument("-out_dir", "--output_directory", type=str, default="~/Data/")
    args = parser.parse_args()

    print("Load event stream ...")
    event_stream = load_event_streams([args.traverse])[0]
    print("Event stream loaded")

    tqdm.write("Adding Ground Truth")
    gt = pd.read_parquet(args.groundtruth_filepath)
    print(gt['t'][0], (event_stream['t'][0]) / 1e6)
    gt['t'] = gt['t'] - (event_stream['t'] / 1e6)
    print(gt['t'])
    print("Ground Truth Loaded")

    im_width, im_height = get_size(event_stream)
    sensor_size = (im_width, im_height, 2)
    num_total_pixels = 346 * 260
    print_duration(event_stream)

    outdir = os.path.join(args.output_directory, os.path.basename(args.traverse).split('.')[0])
    os.makedirs(outdir, exist_ok=True)

    print("Create event frames ...")

    event_frames_raw = [
        tonic.functional.to_frame_numpy(
            event_stream,
            sensor_size,
            time_window=args.event_frame_length * 1e6,
        )
    ]

    event_frames_pos = [event_frame[:, 0, ...] for event_frame in event_frames_raw][0]
    event_frames_neg = [event_frame[:, 1, ...] for event_frame in event_frames_raw][0]
    print("Event frames created")

    event_frame_times = get_times_for_streams_const_time(event_stream, args.event_frame_length * 1e6, 0) / 1e6

    skipped = 0

    for event_frame_pos, event_frame_neg, event_time in tzip(event_frames_pos, event_frames_neg, event_frame_times):
        # event_frame = remove_dead_pixels(event_frame, dead_pixels)
        event_frame = np.zeros((im_height, im_width, 3), dtype=np.uint8)
        
        # for i in range(im_height):
        #     event_frame_pos[i,event_frame_pos[i,:] > np.median(event_frame_pos[i,:])] = 255
        # event_frame_pos[event_frame_pos > 0] = 255
        # event_frame_neg[event_frame_neg > 0] = 255
        event_frame[:, :, 0] = event_frame_pos
        event_frame[:, :, 2] = event_frame_neg

        if ((get_percentage_pixels_with_events(event_frame[:,:,0], num_total_pixels) > args.min_event_threshold) and 
            (get_percentage_pixels_with_events(event_frame[:,:,0], num_total_pixels) < args.max_event_threshold) and
            (get_percentage_pixels_with_events(event_frame[:,:,2], num_total_pixels) > args.min_event_threshold) and 
            (get_percentage_pixels_with_events(event_frame[:,:,2], num_total_pixels) < args.max_event_threshold)):

            # print(get_percentage_pixels_with_events(event_frame, num_total_pixels))
            
            gt['time_difference'] = (gt['t'] - event_time).abs()
            closest_row_index = gt['time_difference'].idxmin()
            closest_row = gt.loc[closest_row_index]
            
            # tqdm.write(f"save {event_time}")
            cv2.imwrite(f"{outdir}/{event_time:07.3f}_{closest_row['x']:07.3f}_{closest_row['y']:07.3f}.png", event_frame)
            # cv2.imwrite(f"{outdir}/{event_time:07.3f}.png", event_frame)

            # print(event_time)
            # event_frame[event_frame > 255] = 255
            # np.save(f"{outdir}/{event_time:03.3f}.npy", event_frame)
        else:
            # tqdm.write("skip frame")
            skipped = skipped + 1

    print(f"Skipped {skipped} out of {event_frames_pos.shape[0]} frames")