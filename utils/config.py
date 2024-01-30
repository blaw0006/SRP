#sample_rate = 32000
sample_rate = 16000
clip_samples = sample_rate * 30

mel_bins = 64
fmin = 50
#fmax = 14000
fmax = 8000
#window_size = 1024
window_size = 512
#hop_size = 320
hop_size = 160
window = 'hann'
pad_mode = 'reflect'
center = True
device = 'cuda'
ref = 1.0
amin = 1e-10
top_db = None

#labels = ['blues', 'classical', 'country', 'disco', 'hiphop', 'jazz', 'metal', 'pop', 'reggae', 'rock']
labels = ['collision', 'no_collision']

lb_to_idx = {lb: idx for idx, lb in enumerate(labels)}
idx_to_lb = {idx: lb for idx, lb in enumerate(labels)}
classes_num = len(labels)