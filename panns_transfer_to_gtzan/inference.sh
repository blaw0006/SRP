#!/usr/bin/bash
CHECKPOINT_PATH="/home/acrv/blaw_ws/workspace/checkpoints/main/holdout_fold=2/Transfer_Cnn14_16k/pretrain=True/loss_type=clip_nll/augmentation=mixup/batch_size=4/freeze_base=False/10000_iterations.pth"   
MODEL_TYPE="Transfer_Cnn14_16k"
CUDA_VISIBLE_DEVICES=0 python3 /home/acrv/blaw_ws/ur5_control/src/SRP/panns_transfer_to_gtzan/pytorch/inference.py audio_tagging \
    --sample_rate=16000 \
    --window_size=512 \
    --hop_size=160 \
    --mel_bins=64 \
    --fmin=50 \
    --fmax=8000 \
    --model_type=$MODEL_TYPE \
    --checkpoint_path=$CHECKPOINT_PATH \
    --audio_path='/home/acrv/blaw_ws/mp3_data/collision/1mic1_test1.mp3' \
    --cuda