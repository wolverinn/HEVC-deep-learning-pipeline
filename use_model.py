import torch
import torch.nn as nn
from torchvision import transforms
import os,shutil
import numpy as np
from PIL import Image
import time
import math

'''
NEXT STEP:
1. Use dataloader to accelerate
2. HEVC每次读入一整帧的预测内容
'''

class ConvNet2(nn.Module):
    def __init__(self):
        super().__init__()
        # (3,32,32)
        self.conv1 = nn.Sequential(
            nn.Conv2d(3,16,5,padding=2),
            nn.BatchNorm2d(16,affine=True),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2)
            ) # (16,16,16)
        self.conv2 = nn.Sequential(
            nn.Conv2d(32,64,3,padding=1),
            nn.BatchNorm2d(64,affine=True),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2)
            ) # (64,8,8)
        self.conv3 = nn.Sequential(
            nn.Conv2d(64,128,3,padding=1),
            nn.BatchNorm2d(128,affine=True),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=2)
            ) # (128,4,4)
        self.fc1 = nn.Sequential(nn.Linear(128*4*4,256),nn.ReLU())
        self.fc2 = nn.Sequential(nn.Linear(256,64),nn.ReLU())
        self.fc3 = nn.Linear(64,16)
        self.conv64 = nn.Sequential(
            nn.Conv2d(3,16,5,padding=2),
            nn.BatchNorm2d(16,affine=True),
            nn.ReLU(),
            nn.MaxPool2d(kernel_size=4)
            ) # (16,16,16) -> (64,16,16)
        # self.dropout = nn.Dropout(0.25)
    def forward(self,x32,x64):
        in_size = x32.size(0)
        out = torch.cat([self.conv1(x32),self.conv64(x64)],dim=1)
        out = self.conv2(out)
        out = self.conv3(out)
        out = out.view(in_size,-1) # 扁平化flat然后传入全连接层
        out = self.fc1(out)
        # out = self.dropout(out)
        out = self.fc2(out)
        out = self.fc3(out)
        return out

DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu") # 让torch判断是否使用GPU
model = ConvNet2().to(DEVICE)
model.load_state_dict(torch.load('./rec/hevc_encoder_model.pt',map_location=DEVICE))
print("loaded model from drive")

with open("bitstream.cfg",'r') as f:
    for i,line in enumerate(f):
        line = line.split(":")
        if i == 7:
            frame_tobe_encoded = line[1].strip(" ").strip('\n')
        else:
            pass

total_frames = len(list(os.listdir("./rec/frames")))
for frame_number in range(1,total_frames+1):
    if frame_number > int(frame_tobe_encoded):
        break
    os.mkdir("./pred/{}".format(frame_number-1))
    img = Image.open("./rec/frames/{}.jpg".format(frame_number))
    img_width, img_height = img.size
    ctu_numbers = math.ceil(img_width / 64) * math.ceil(img_height / 64)
    label = []
    for i in range(16):
        label.append(str(i))

    with torch.no_grad():
        for i in range(ctu_numbers):
            img_row = i // math.ceil(img_width / 64)
            img_colonm = i % math.ceil(img_width / 64)
            for layer2 in range(4):
                start_pixel_x = img_colonm * 64 + (layer2 % 2)*32
                start_pixel_y = img_row * 64 + (layer2 // 2)*32
                cropped_img32 = img.crop((start_pixel_x, start_pixel_y, start_pixel_x + 32, start_pixel_y + 32))
                cropped_img64 = img.crop((img_colonm * 64, img_row * 64, img_colonm * 64 + 64, img_row * 64 + 64))
                data32 = transforms.ToTensor()(cropped_img32).unsqueeze(0)
                data64 = transforms.ToTensor()(cropped_img64).unsqueeze(0)
                cropped_img32.close()
                cropped_img64.close()
                data32 = data32.to(DEVICE)
                data64 = data64.to(DEVICE)
                output = model(data32,data64)
                pred = str(int(torch.argmax(output[0,0:4]))) + str(int(torch.argmax(output[0,4:8]))) + str(int(torch.argmax(output[0,8:12]))) + str(int(torch.argmax(output[0,12:16])))
                if "0" in pred and pred != "0000":
                    pred = pred.replace("0","1")
                if "1" in pred and pred != "1111":
                    pred = pred.replace("1","2")
                if layer2 == 0:
                    label[0],label[1],label[4],label[5] = pred[0],pred[1],pred[2],pred[3]
                elif layer2 == 1:
                    if pred == "0000" and label[0] != "0":
                        pred = "1111"
                    label[2],label[3],label[6],label[7] = pred[0],pred[1],pred[2],pred[3]
                elif layer2 == 2:
                    if pred == "0000" and label[2] != "0":
                        pred = "1111"
                    label[8],label[9],label[12],label[13] = pred[0],pred[1],pred[2],pred[3]
                else:
                    if pred == "0000" and label[8] != "0":
                        pred = "1111"
                    label[10],label[11],label[14],label[15] = pred[0],pred[1],pred[2],pred[3]
            # print(label)
            with open("./pred/{}/ctu.txt".format(frame_number-1),'w',encoding='utf-8') as f:
                for m in range(16):
                    f.write(label[m])
                    f.write(" ")
            os.rename("./pred/{}/ctu.txt".format(frame_number-1),"./pred/{}/ctu{}.txt".format(frame_number-1,i))
    img.close()
    os.remove("./rec/frames/{}.jpg".format(frame_number))