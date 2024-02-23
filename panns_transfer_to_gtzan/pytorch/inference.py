import os
import re
import sys
sys.path.insert(1, os.path.join(sys.path[0], '../utils'))
import numpy as np
import argparse
import librosa
#import ffmpeg
import matplotlib.pyplot as plt
import torch

from utilities import create_folder, get_filename
#from models import *
from pytorch_utils import move_data_to_device
import config
from models_new import (Cnn14, Cnn14_no_specaug, Cnn14_no_dropout, 
    Cnn6, Cnn10, ResNet22, ResNet38, ResNet54, Cnn14_emb512, Cnn14_emb128, 
    Cnn14_emb32, MobileNetV1, MobileNetV2, LeeNet11, LeeNet24, DaiNet19, 
    Res1dNet31, Res1dNet51, Wavegram_Cnn14, Wavegram_Logmel_Cnn14, 
    Wavegram_Logmel128_Cnn14, Cnn14_16k, Cnn14_8k, Cnn14_mel32, Cnn14_mel128, 
    Cnn14_mixup_time_domain, Cnn14_DecisionLevelMax, Cnn14_DecisionLevelAtt, Transfer_Cnn14_16k) # NOTE: added this so the new model can be loaded


def audio_tagging(args):
    """Inference audio tagging result of an audio clip.
    """

    # Arugments & parameters
    sample_rate = args.sample_rate
    window_size = args.window_size
    hop_size = args.hop_size
    mel_bins = args.mel_bins
    fmin = args.fmin
    fmax = args.fmax
    model_type = args.model_type # Transfer_cnn14_16k
    #freeze_base = args.freeze_base # added this so it accepts this argument
    checkpoint_path = args.checkpoint_path
    audio_path = args.audio_path
    device = torch.device('cuda') if args.cuda and torch.cuda.is_available() else torch.device('cpu')
    
    classes_num = config.classes_num
    labels = config.labels

    # Model
    Model = eval(model_type)
    model = Model(sample_rate=sample_rate, window_size=window_size, 
        hop_size=hop_size, mel_bins=mel_bins, fmin=fmin, fmax=fmax, 
        classes_num=classes_num, freeze_base=True)
    
    checkpoint = torch.load(checkpoint_path, map_location=device) # load trained model
    model.load_state_dict(checkpoint['model']) # error loading state dict 

    # Parallel
    if 'cuda' in str(device):
        model.to(device)
        print('GPU number: {}'.format(torch.cuda.device_count()))
        model = torch.nn.DataParallel(model)
    else:
        print('Using CPU.')
    
    # Get files in specified folder
    files = os.listdir(audio_path)
    object_files = [[] for _ in range(9)]
    drop_height_files = [[] for _ in range(4)]
    drop_location_files = [[] for _ in range(17)]


    ##### Apply filters to get certain files ######
    ### Filter files per object ###
    for filename in files:
        for i in range(1,10): # add file to correct inner list based on object number
            if f"object{i}" in filename:
                object_files[i-1].append(os.path.join(audio_path, filename))
    
    print("##### Object files #####")
    for i, inner_list in enumerate(object_files):
        print(f"Files for object {i+1}:")
        print(len(inner_list))

    
    ### Filter files per drop height ###
    pattern = r'_test(\d+)_'
    for filename in files: 
        match = re.search(pattern, filename)
        if match:
            test_number = int(match.group(1))
            
            # Calculate starting test number
            object_test = (int(test_number) - 1) % 68 + 1

            # Add file to correct list based on drop height (test number)
            if object_test <= 17:
                drop_height_files[0].append(os.path.join(audio_path, filename))
            elif object_test <= 34:
                drop_height_files[1].append(os.path.join(audio_path, filename))
            elif object_test <= 51:
                drop_height_files[2].append(os.path.join(audio_path, filename))
            elif object_test <= 68:
                drop_height_files[3].append(os.path.join(audio_path, filename))

    print("##### Drop height files #####")

    for i, inner_list in enumerate(drop_height_files):
        print(f"Files for drop height {i+1}:")
        print(len(inner_list))


    ### Filter files for drop location ###
    for filename in files:
        # Extract the test number from the filename
        match = re.search(pattern, filename)
        if match:
            test_number = int(match.group(1))
        
            # Determine the drop location based on the test number
            if test_number % 17 == 0:
                drop_location = 16
            else:
                drop_location = test_number % 17 - 1

            # if drop_location == 0:
            #     print(test_number)

            # Add the file to the correct inner list
            drop_location_files[drop_location].append(os.path.join(audio_path, filename))

    print("### Drop location ###")

    for i, inner_list in enumerate(drop_location_files):
        print(f"Files for drop location {i+1}:")
        print(len(inner_list))


    ##### Test model #####
    ### Compare objects ###
    object_accuracy = []
    height_accuracy = []
    location_accuracy = []
    object_number = 1
    total = 0
    correct_predictions = 0
    outputs = []

    for inner_list in object_files: # change object_files to whatever data you need, the rest of the loop can stay the same
        correct_predictions = 0
        total = 0

        for file_path in inner_list:
            try:
                (waveform, _) = librosa.load(file_path, sr=sample_rate, mono=True)
                waveform = waveform[None, :]    # (1, audio_length)
                waveform = move_data_to_device(waveform, device)

                # Forward
                with torch.no_grad():
                    model.eval()
                    batch_output_dict = model(waveform, None) 

                clipwise_output = batch_output_dict['clipwise_output'].data.cpu().numpy()[0]
                """(classes_num,)"""

                #sorted_indexes = np.argsort(clipwise_output)[::-1]
                
                # clipwise_output[1] is collision probability, clipwise_output[0] is no_collision probability
                # if prob(collision) > prob(no_collision) and collision==True
                if clipwise_output[1] > clipwise_output[0] and os.path.basename(file_path).startswith("1"):
                    correct_predictions += 1
                # if prob(no_collision) > prob(collision) and no_collision==True
                elif clipwise_output[0] > clipwise_output[1] and os.path.basename(file_path).startswith("0"):
                    correct_predictions += 1
                # Print audio tagging top probabilities
                # for k in range(2): # altered to two since there are only two possible outputs (unlike the 500 for AudioNet)
                #     print('{}: {:.3f}'.format(np.array(labels)[sorted_indexes[k]], 
                #         np.exp(clipwise_output[k]))
                #         )

                total += 1


                # Print embedding
                # if 'embedding' in batch_output_dict.keys():
                #     embedding = batch_output_dict['embedding'].data.cpu().numpy()[0]
                #     print('embedding: {}'.format(embedding.shape))
            except Exception as e:
                print(e)
                continue
        
        # Save accuracy for that object
        print(correct_predictions)
        print(total)
        accuracy = correct_predictions/total
        object_accuracy.append(accuracy)
        print(f"Object number: {object_number}")
        print(f"Accuracy: {accuracy}")
        object_number += 1

    
    return clipwise_output, labels


def sound_event_detection(args):
    """Inference sound event detection result of an audio clip.
    """

    # Arugments & parameters
    sample_rate = args.sample_rate
    window_size = args.window_size
    hop_size = args.hop_size
    mel_bins = args.mel_bins
    fmin = args.fmin
    fmax = args.fmax
    model_type = args.model_type
    checkpoint_path = args.checkpoint_path
    audio_path = args.audio_path
    device = torch.device('cuda') if args.cuda and torch.cuda.is_available() else torch.device('cpu')

    classes_num = config.classes_num
    labels = config.labels
    frames_per_second = sample_rate // hop_size

    # Paths
    fig_path = os.path.join('results', '{}.png'.format(get_filename(audio_path)))
    create_folder(os.path.dirname(fig_path))

    # Model
    Model = eval(model_type)
    model = Model(sample_rate=sample_rate, window_size=window_size, 
        hop_size=hop_size, mel_bins=mel_bins, fmin=fmin, fmax=fmax, 
        classes_num=classes_num)
    
    checkpoint = torch.load(checkpoint_path, map_location=device)
    model.load_state_dict(checkpoint['model'])

    # Parallel
    print('GPU number: {}'.format(torch.cuda.device_count()))
    model = torch.nn.DataParallel(model)

    if 'cuda' in str(device):
        model.to(device)
    
    # Load audio
    (waveform, _) = librosa.core.load(audio_path, sr=sample_rate, mono=True)

    waveform = waveform[None, :]    # (1, audio_length)
    waveform = move_data_to_device(waveform, device)

    # Forward
    with torch.no_grad():
        model.eval()
        batch_output_dict = model(waveform, None)

    framewise_output = batch_output_dict['framewise_output'].data.cpu().numpy()[0]
    """(time_steps, classes_num)"""

    print('Sound event detection result (time_steps x classes_num): {}'.format(
        framewise_output.shape))

    sorted_indexes = np.argsort(np.max(framewise_output, axis=0))[::-1]

    top_k = 10  # Show top results
    top_result_mat = framewise_output[:, sorted_indexes[0 : top_k]]    
    """(time_steps, top_k)"""

    # Plot result    
    stft = librosa.core.stft(y=waveform[0].data.cpu().numpy(), n_fft=window_size, 
        hop_length=hop_size, window='hann', center=True)
    frames_num = stft.shape[-1]

    fig, axs = plt.subplots(2, 1, sharex=True, figsize=(10, 4))
    axs[0].matshow(np.log(np.abs(stft)), origin='lower', aspect='auto', cmap='jet')
    axs[0].set_ylabel('Frequency bins')
    axs[0].set_title('Log spectrogram')
    axs[1].matshow(top_result_mat.T, origin='upper', aspect='auto', cmap='jet', vmin=0, vmax=1)
    axs[1].xaxis.set_ticks(np.arange(0, frames_num, frames_per_second))
    axs[1].xaxis.set_ticklabels(np.arange(0, frames_num / frames_per_second))
    axs[1].yaxis.set_ticks(np.arange(0, top_k))
    axs[1].yaxis.set_ticklabels(np.array(labels)[sorted_indexes[0 : top_k]])
    axs[1].yaxis.grid(color='k', linestyle='solid', linewidth=0.3, alpha=0.3)
    axs[1].set_xlabel('Seconds')
    axs[1].xaxis.set_ticks_position('bottom')

    plt.tight_layout()
    plt.savefig(fig_path)
    print('Save sound event detection visualization to {}'.format(fig_path))

    return framewise_output, labels


if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Example of parser. ')
    subparsers = parser.add_subparsers(dest='mode')

    parser_at = subparsers.add_parser('audio_tagging')
    parser_at.add_argument('--sample_rate', type=int, default=32000)
    parser_at.add_argument('--window_size', type=int, default=1024)
    parser_at.add_argument('--hop_size', type=int, default=320)
    parser_at.add_argument('--mel_bins', type=int, default=64)
    parser_at.add_argument('--fmin', type=int, default=50)
    parser_at.add_argument('--fmax', type=int, default=14000) 
    parser_at.add_argument('--model_type', type=str, required=True)
    #parser_at.add_argument('--freeze_base', action='store_true', default=False) # added this cos missing freeze_base argument
    parser_at.add_argument('--checkpoint_path', type=str, required=True)
    parser_at.add_argument('--audio_path', type=str, required=True)
    parser_at.add_argument('--cuda', action='store_true', default=False)


    parser_sed = subparsers.add_parser('sound_event_detection')
    parser_sed.add_argument('--sample_rate', type=int, default=32000)
    parser_sed.add_argument('--window_size', type=int, default=1024)
    parser_sed.add_argument('--hop_size', type=int, default=320)
    parser_sed.add_argument('--mel_bins', type=int, default=64)
    parser_sed.add_argument('--fmin', type=int, default=50)
    parser_sed.add_argument('--fmax', type=int, default=14000) 
    parser_sed.add_argument('--model_type', type=str, required=True)
    #parser_sed.add_argument('--freeze_base', action='store_true', default=False) # added here as well
    parser_sed.add_argument('--checkpoint_path', type=str, required=True)
    parser_sed.add_argument('--audio_path', type=str, required=True)
    parser_sed.add_argument('--cuda', action='store_true', default=False)
    
    args = parser.parse_args()

    if args.mode == 'audio_tagging':
        audio_tagging(args)

    elif args.mode == 'sound_event_detection':
        sound_event_detection(args)

    else:
        raise Exception('Error argument!')