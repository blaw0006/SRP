#!/usr/bin/env python3

import os
import random
import shutil

def move_files(training_folder, validation_folder):
    # Get a list of all subfolders in the Training folder
    subfolders = [subfolder for subfolder in os.listdir(training_folder) if os.path.isdir(os.path.join(training_folder, subfolder))]

    total_validation_files = 0

    # Iterate over each subfolder
    for subfolder in subfolders:
        subfolder_path = os.path.join(training_folder, subfolder)
        training_files = os.listdir(subfolder_path)

        # Calculate the number of files to move to the Validation folder from this subfolder (20%)
        num_validation_files = int(len(training_files) * 0.2)

        # Randomly select files for validation from this subfolder
        validation_files = random.sample(training_files, num_validation_files)

        # Move selected files to the Validation folder
        for file_name in validation_files:
            # Construct the full paths for source and destination files
            src = os.path.join(subfolder_path, file_name)
            dst = os.path.join(validation_folder, subfolder, file_name)

            # Move the file
            shutil.move(src, dst)

        total_validation_files += num_validation_files

    print(f"{total_validation_files} files moved to the Validation folder.")

if __name__ == '__main__':
    training_folder = '/home/acrv/blaw_ws/ur5_control/src/SRP/src/4_mic_data/mp3_data/Training'
    validation_folder = '/home/acrv/blaw_ws/ur5_control/src/SRP/src/4_mic_data/mp3_data/Validation'
    move_files(training_folder, validation_folder)
