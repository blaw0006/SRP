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


def move_files_with_term(source_dir, target_dir, term):
    # Initialize counter for moved files
    moved_files_count = 0

    # Iterate through files in source_dir
    for root, dirs, files in os.walk(source_dir):
        for file in files:
            # Check if the file name contains the term
            if term in file:
                # Construct source and target paths
                src_path = os.path.join(root, file)
                dst_path = os.path.join(target_dir, file)
                # Move the file to the target directory
                shutil.move(src_path, dst_path)
                # Increment moved files count
                moved_files_count += 1

    # Print the number of files moved
    print(f"Number of files moved: {moved_files_count}")


if __name__ == '__main__':
    source_dir = '/home/acrv/blaw_ws/ur5_control/src/SRP/src/4_mic_data/mp3_data/Training'
    target_dir = '/home/acrv/blaw_ws/ur5_control/src/SRP/src/4_mic_data/mp3_data/mic1_data'
    term = 'mic1'    
    move_files_with_term(source_dir, target_dir, term)

    training_folder = '/home/acrv/blaw_ws/ur5_control/src/SRP/src/4_mic_data/mp3_data/Training'
    validation_folder = '/home/acrv/blaw_ws/ur5_control/src/SRP/src/4_mic_data/mp3_data/Validation'

    #move_files(training_folder, validation_folder)
