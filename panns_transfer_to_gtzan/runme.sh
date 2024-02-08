#!/usr/bin/bash
#DATASET_DIR="/home/caitlin/blaw_ws/src/ur5_control/src/SRP/src/wav_files" # assumes that the directory is setup with the labels (contains a 'collision' and 'no collision' folder)
#DATASET_DIR="/home/acrv/blaw_ws/src/ur5_control/src/SRP/src/wav_files"
#DATASET_DIR="/home/acrv/blaw_ws/ur5_control/src/SRP/src/mp3_test"
DATASET_DIR="/home/acrv/blaw_ws/mp3_data"

#WORKSPACE="/home/caitlin/blaw_ws/src/panns_transfer_to_gtzan"
#WORKSPACE="/home/acrv/blaw_ws/src/ur5_control/src/SRP/panns_transfer_to_gtzan"
WORKSPACE="/home/acrv/blaw_ws/workspace" # store checkpoints and logs outside git repo since they are very large

python3 ../panns_transfer_to_gtzan/utils/features.py pack_audio_files_to_hdf5 --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE 


#PRETRAINED_CHECKPOINT_PATH="/home/caitlin/blaw_ws/src/panns_transfer_to_gtzan/workspace/Cnn14_16k_mAP=0.438.pth"
PRETRAINED_CHECKPOINT_PATH="/home/acrv/blaw_ws/pretrained/Cnn14_16k_mAP=0.438.pth" # locally saved file, since lfs is causing issues


#CUDA_VISIBLE_DEVICES=3 python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=1 --model_type="Transfer_Cnn14" --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=32 --resume_iteration=0 --stop_iteration=10000 --cuda
python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=2 --model_type="Transfer_Cnn14_16k" --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=2 --resume_iteration=0 --stop_iteration=10000 --cuda


#####
#MODEL_TYPE="Transfer_Cnn13"
#PRETRAINED_CHECKPOINT_PATH="/vol/vssp/msos/qk/bytedance/workspaces_important/pub_audioset_tagging_cnn_transfer/checkpoints/main/sample_rate=32000,window_size=1024,hop_size=320,mel_bins=64,fmin=50,fmax=14000/data_type=full_train/Cnn13/loss_type=clip_bce/balanced=balanced/augmentation=mixup/batch_size=32/660000_iterations.pth"
#python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=1 --model_type=$MODEL_TYPE --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --freeze_base --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=32 --few_shots=10 --random_seed=1000 --resume_iteration=0 --stop_iteration=10000 --cuda

#python3 utils/plot_statistics.py 1 --workspace=$WORKSPACE --select=2_cnn13





