import os
import sys
import numpy as np
import argparse
import h5py
import librosa
import matplotlib.pyplot as plt
import time
import csv
import math
import re
import random

import config
from utilities import create_folder, traverse_folder, float32_to_int16


def to_one_hot(k, classes_num):
    target = np.zeros(classes_num)
    target[k] = 1
    return target


def pad_truncate_sequence(x, max_len):
    if len(x) < max_len:
        return np.concatenate((x, np.zeros(max_len - len(x))))
    else:
        return x[0 : max_len]


def pack_audio_files_to_hdf5(args):
    '''
    Function explores specified data folder, extracts data, and packs them to a single very large hdf5 waveform folder.
    This is a hierarchical data structure file format that organises and stores the data, almost like a folder embedded as a file.
    '''
    # Arguments & parameters
    dataset_dir = args.dataset_dir
    workspace = args.workspace
    mini_data = args.mini_data

    sample_rate = config.sample_rate
    clip_samples = config.clip_samples
    classes_num = config.classes_num
    lb_to_idx = config.lb_to_idx # contains the label names 

    # Paths
    audios_dir = os.path.join(dataset_dir) # "/home/caitlin/blaw_ws/src/ur5_control/src/SRP/src/pngs"

    if mini_data:
        # destination directory for the converted hdf5 files later
        packed_hdf5_path = os.path.join(workspace, 'features', 'minidata_waveform.h5')
    else:
        packed_hdf5_path = os.path.join(workspace, 'features', 'waveform.h5')
    create_folder(os.path.dirname(packed_hdf5_path))

    (audio_names, audio_paths) = traverse_folder(audios_dir) # recursively explores all folders to extract filenames and paths 
    
    audio_names = sorted(audio_names) # contains names of files including .mp3
    audio_paths = sorted(audio_paths) # contains path to files (within the label folders)

    # check the audio names in the directory
    # for i in audio_names:
    #     print(i)
    #     print(i.split('.')[0]) # this just splits the extension from the filename

    # for i in audio_paths:
    #     print(i)
    
    # # check the keys in lb_to_idx
    # print("Keys in lb_to_idx:", list(lb_to_idx.keys()))
    # print(np.array([lb_to_idx[audio_name.split('.')[0]] for audio_name in audio_names]))


    # creates a dictionary with 4 key-value pairs 
    meta_dict = {
        'audio_name': np.array(audio_names), 
        'audio_path': np.array(audio_paths), 

        # target is meant to be an index corresponding to the label of that spectrogram image
        #'target': np.array([lb_to_idx[audio_name.split('.')[0]] for audio_name in audio_names]), 
        'target': np.array([int(audio_name[0]) for audio_name in audio_names]),
        'fold': np.arange(len(audio_names)) % 10 + 1}
    
    if mini_data:
        mini_num = 10
        total_num = len(meta_dict['audio_name'])
        random_state = np.random.RandomState(1234)
        indexes = random_state.choice(total_num, size=mini_num, replace=False)
        for key in meta_dict.keys():
            meta_dict[key] = meta_dict[key][indexes]

    audios_num = len(meta_dict['audio_name'])

    feature_time = time.time()
    with h5py.File(packed_hdf5_path, 'w') as hf:
        hf.create_dataset(
            name='audio_name', 
            shape=(audios_num,), 
            dtype='S80')

        hf.create_dataset(
            name='waveform', 
            shape=(audios_num, clip_samples), 
            dtype=np.int16)

        hf.create_dataset(
            name='target', 
            shape=(audios_num, classes_num), 
            dtype=np.float32)

        hf.create_dataset(
            name='fold', 
            shape=(audios_num,), 
            dtype=np.int32)
 
        for n in range(audios_num):
            try:
                print(n) # this is the line printing file numbers that have been converted. Halts at around 500 - has issue around clip 500.
                audio_name = meta_dict['audio_name'][n]
                fold = meta_dict['fold'][n]
                audio_path = meta_dict['audio_path'][n]
                #(audio, fs) = librosa.core.load(audio_path, sr=sample_rate, mono=True)
                (audio, fs) = librosa.load(audio_path, sr=sample_rate, mono=True) # specific line that is causing issues - update: upgrading package fixed it

                audio = pad_truncate_sequence(audio, clip_samples)

                hf['audio_name'][n] = audio_name.encode()
                hf['waveform'][n] = float32_to_int16(audio)
                hf['target'][n] = to_one_hot(meta_dict['target'][n], classes_num)
                hf['fold'][n] = meta_dict['fold'][n]
            except Exception as e:
                print(e)
                continue

    print('Write hdf5 to {}'.format(packed_hdf5_path))
    print('Time: {:.3f} s'.format(time.time() - feature_time))


if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description='')
    subparsers = parser.add_subparsers(dest='mode')

    # Calculate feature for all audio files
    parser_pack_audio = subparsers.add_parser('pack_audio_files_to_hdf5')
    parser_pack_audio.add_argument('--dataset_dir', type=str, required=True, help='Directory of dataset.')
    parser_pack_audio.add_argument('--workspace', type=str, required=True, help='Directory of your workspace.')
    parser_pack_audio.add_argument('--mini_data', action='store_true', default=False, help='Set True for debugging on a small part of data.')
    
    # Parse arguments
    args = parser.parse_args()
    
    if args.mode == 'pack_audio_files_to_hdf5':
        pack_audio_files_to_hdf5(args)
        
    else:
        raise Exception('Incorrect arguments!')