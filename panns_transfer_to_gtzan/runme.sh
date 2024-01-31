#!/usr/bin/bash
#DATASET_DIR="/home/caitlin/blaw_ws/src/ur5_control/src/SRP/src/wav_files" # assumes that the directory is setup with the labels (contains a 'collision' and 'no collision' folder)
DATASET_DIR="/home/acrv/blaw_ws/src/ur5_control/src/SRP/src/wav_files"
#WORKSPACE="/home/caitlin/blaw_ws/src/panns_transfer_to_gtzan"
WORKSPACE="/home/acrv/blaw_ws/src/ur5_control/src/SRP/panns_transfer_to_gtzan"


python3 ../panns_transfer_to_gtzan/utils/features.py pack_audio_files_to_hdf5 --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE 


#PRETRAINED_CHECKPOINT_PATH="/home/caitlin/blaw_ws/src/panns_transfer_to_gtzan/workspace/Cnn14_16k_mAP=0.438.pth"
PRETRAINED_CHECKPOINT_PATH="/home/acrv/blaw_ws/src/ur5_control/src/SRP/panns_transfer_to_gtzan/workspace/Cnn14_16k_mAP=0.438.pth"


#CUDA_VISIBLE_DEVICES=3 python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=1 --model_type="Transfer_Cnn14" --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=32 --resume_iteration=0 --stop_iteration=10000 --cuda
python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=1 --model_type="Transfer_Cnn14_16k" --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=2 --resume_iteration=0 --stop_iteration=10000 --cuda


#####
#MODEL_TYPE="Transfer_Cnn13"
#PRETRAINED_CHECKPOINT_PATH="/vol/vssp/msos/qk/bytedance/workspaces_important/pub_audioset_tagging_cnn_transfer/checkpoints/main/sample_rate=32000,window_size=1024,hop_size=320,mel_bins=64,fmin=50,fmax=14000/data_type=full_train/Cnn13/loss_type=clip_bce/balanced=balanced/augmentation=mixup/batch_size=32/660000_iterations.pth"
#python3 pytorch/main.py train --dataset_dir=$DATASET_DIR --workspace=$WORKSPACE --holdout_fold=1 --model_type=$MODEL_TYPE --pretrained_checkpoint_path=$PRETRAINED_CHECKPOINT_PATH --freeze_base --loss_type=clip_nll --augmentation='mixup' --learning_rate=1e-4 --batch_size=32 --few_shots=10 --random_seed=1000 --resume_iteration=0 --stop_iteration=10000 --cuda

#python3 utils/plot_statistics.py 1 --workspace=$WORKSPACE --select=2_cnn13


# Having a path issue. Having the ~/ paths does not work, it simply creates that path in the current directory.
# Try running from home ?
# figure out the file paths and how to train properly - does it need wav or mel spectrogram input?
# collect proper dataset for training
# Note: moved this runme file from panns_transfer_to_gtzan to under ur5_control so that it can be pusehd to git

# either convert files to hdf5 myself for input
# or convert to png and use their functions


