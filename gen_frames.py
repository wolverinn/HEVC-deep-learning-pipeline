import os
import shutil

with open("bitstream.cfg",'r') as f:
    for i,line in enumerate(f):
        line = line.split(":")
        if i == 0:
            yuv_file = ":".join(line[1:]).strip(" ").strip('\n')
        elif i == 3:
            frame_rate = line[1].strip(" ").strip('\n')
        elif i == 5:
            yuv_width = line[1].strip(" ").strip('\n')
        elif i == 6:
            yuv_height = line[1].strip(" ").strip('\n')
        else:
            pass
try:
    os.mkdir("./rec/frames")
except:
    pass
gen_frames_cmd = "ffmpeg -video_size {}x{} -r {} -pixel_format yuv420p -i {} .\\rec\\frames\\%d.jpg".format(yuv_width,yuv_height,frame_rate,yuv_file)
os.system(gen_frames_cmd)
try:
    shutil.rmtree("./pred")
except:
    pass
os.mkdir("./pred")