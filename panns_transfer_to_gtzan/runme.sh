#!/usr/bin/bash
DATASET_DIR="/home/acrv/blaw_ws/ur5_control/src/SRP/src/4_mic_data/mp3_data"

WORKSPACE="/home/acrv/blaw_ws/workspace_4mic" 

python3 ../panns_transfer_to_gtzan/utils/features.py pack_audio_files_to_hdf5 --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE 


PRETRAINED_CHECKPOINT_PATH="/home/acrv/blaw_ws/pretrained/Cnn14_16k_mAP=0.438.pth" # locally saved file, since lfs is causing issues


#CUDA_VISIBLE_DEVICES=3 python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=1 --model_type="Transfer_Cnn14" --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=32 --resume_iteration=0 --stop_iteration=10000 --cuda
python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=4 --model_type="Transfer_Cnn14_16k" --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=8 --resume_iteration=0 --stop_iteration=400 --cuda






