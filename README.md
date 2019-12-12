# HEVC-deep-learning-pipeline
Integrating neural network models in HEVC encoder, to test the complexity reduction using deep-learning-based method in HEVC intra-prediction.

## Introduction
Using neural networks, we can directly predict the Coding Unit (CU) depths for each frame. The intention is to speed up the encoding process of HEVC encoder. Thus, after we have a trained model, another thing that needs to be done is to integrate the deep learning prediction process into the HEVC encoder. Not only is this a pratical thing but also it makes the evaluation of the performance of our neural network model easier.

## Usage
This pipeline is used for evaluating the performance of a neural network model in HEVC intra-prediction process. Compare the difference in encoding time, BDBR, BD-PSNR with the original HEVC encoder.

Requirements:
- FFmpeg
- Python3
- PyTorch

First, prepare at least four [YUV test sequence](http://www.ucodec.com/resources.html), but you can only encode one YUV at a time. Modify the ```bitstream.cfg``` according to the properties of the YUV file. Then, build the deep-learning-based HEVC encoder from source in folder ```HM_dl```, move the ```TAppEncoder.exe``` or executables for other platforms to the repository directory, and use

```
.\TAppEncoder.exe -c .\\encoder_intra_main.cfg -c .\\bitstream.cfg
```

Then the encoding of the YUV file using deep learning model with begin. When it's done, it will output some information on the command line, like the encoding time, YUV-PSNR and so on.

To test the performance, you need to also use the original HEVC encoder on the YUV file and get the information it outputs on the command line. When you have done testing at least four YUV files with both encoders. You can use the information and calculate the BDBR and BD-PSNR. Refer to ```./calc_BDBR``` for details.

## Pipeline description
First, before the HEVC encoder actually begins the encoding process, it triggers ```gen_frames.py```. This python script will read the ```bitstream.cfg```, get the YUV file information, and use ```ffmpeg``` command to extract all the frames of the YUV file that is about to be encoded to the ```./rec/frames``` folder for the prediction of the neural network. The script also creates an empty ```./pred``` folder to store the prediction result (the CU depths information) of the neural network.

After the execution of ```gen_frames.py``` is done. A thread is called to execute ```use_model.py```. Then , the thread is detached and HEVC starts encoding.

```use_model.py``` uses the neural network model in [wolverinn/HEVC-CU-depths-prediction-CNN](https://github.com/wolverinn/HEVC-CU-depths-prediction-CNN). It reads frames from ```./rec/frames``` and processs them. And then use the neural network trained model in ```./rec``` to preedict CU depths from the input. For each 64x64 CTU, the script ouputs a ```.txt``` file to store the depth information in an 1x16 vector (also refer to the previous repository to see why 16 labels are enough to represent CU depths information). The file is stored like ```./pred/FrameNumber/CtuNumber.txt```.

Back to the HEVC encoder side, when the encoder starts to encode a 64x64 CTU in the xth frame, it first checks whether the file ```./pred/x/CtuNumber.txt``` exists. This check is necessary because the prediction and the encoding process are running paralelly. If not, it waits. If the check passes, it reads the depth information, a 16 vector from the txt file.

Then, when the encoder searches recursively in each CU for the best partition. In each depth, it compares the current depth with the predicted depth from the neural network. If ```current_depth < predicted_depth```, then it skips the calculation for current depth and continues to search for next depth. If ```current_depth = predicted_depth```, then it only does calculation for current depth and stops searching further depths. If ```current_depth < predicted_depth```, then it not only skips the current depth calculation, but also stops searching further depths.

This makes sure that calculation will happen only if the current depth is the predicted depth from the neural network model, which achieves the purpose of saving time by skipping unnecessary calculation for RD-cost.

## Modification of HEVC source code
We modified the HEVC trunk source code to make our pipeline. Only the encoder part was modified. Search **modified2019** in the project source code for detailed modification.

- /source/App/TAppEncoder/encmain.cpp
- /TLibCommon/TComPicYuv.h: add member variable ```m_iFrameRcvd``` and member function ```setFrame``` and ```getFrame``` to store and get FrameNumber information in class ```TComPicYuv```.
- /source/App/TAppEncoder/TAppEncTop.cpp: store the current frame number ```m_iFrameRcvd``` in class ```TComPicYuv pcPicYuvOrg```.
- /TLibEncoder/TEncGop.h: add a parameter ```Int m_iFrame``` for member function ```compressGOP```.
- /TLibEncoder/TEncTop.cpp: extract current frame number from ```pcPicYuvOrg``` to variable ```m_iFrame```, pass the variable to the function ```m_cGOPEncoder.compressGOP```. Note: ```m_cGOPEncoder.compressGOP``` exists in two places.
- /TLibEncoder/TEncSlice.h: add a parameter ```Int m_iFrame``` for member function ```compressSlice``` and ```precompressSlice```.
- /TLibEncoder/TEncGop.cpp: add a parameter ```Int m_iFrame``` for function ```compressGOP``` (consistant with the declaration in TEncGop.h). Pass the variable ```m_iFrame``` to function ```m_pcSliceEncoder->precompressSlice``` and ```m_pcSliceEncoder->compressSlice```.
- /TLibEncoder/TEncCu.h: add a parameter ```Int m_iFrame``` for member function ```compressCtu```.
- /TLibEncoder/TEncSlice.cpp: add a parameter ```Int m_iFrame``` for function ```precompressSlice``` and ```compressSlice``` (consistant with the declaration in TEncSlice.h). Note: ```compressSlice``` exists in two places. Pass the variable ```m_iFrame``` to function ```compressCtu```.
- /TLibEncoder/TEncCu.cpp: add a parameter ```Int m_iFrame``` for function ```compressCtu``` (consistant with the declaration in TEncCu.h).
- /TLibCommon/TComDataCu.h: add member variable ```pred_depth``` and member function ```set_pred``` and ```get_pred``` to store and get depth information in class ```TComDataCU```.

#### TEncCu.cpp -- function compressCtu()
use ```m_iFrame``` to get the frame number, use ```pCtu->getCtuRsAddr()``` to get the current ctu number, and read depth prediction from file ```./pred/FrameNumber/CtuNumber.txt``` (availability check first).

Store the depth information in ```m_ppcBestCU[0]->pred_depth```. Use member function ```m_ppcBestCU[0]->set_pred(UInt *label)```.

#### TEncCu.cpp -- function xCompressCU()
use member function ```m_ppcBestCU[0]->get_pred()``` to get the predicted CU depth. Use variable ```uiLPelX``` and ```uiTPelY``` to locate current CU location and get the corresponding depth. Compare the predicted depth to ```uiDepth```. Determine two things:
if the current depth needs calculation/ if the next depth needs calculation.

If current_calculation is False, then skip ```xCheckRDCostIntra```. Instead, directly assign:

```cpp
rpcBestCU->getTotalCost() = MAX_DOUBLE / 16;
rpcBestCU->getTotalDistortion() = MAX_UINT >> 3;
rpcBestCU->getTotalBits() = MAX_UINT >> 3;
rpcBestCU->setPartitionSize(0, SIZE_2Nx2N);
rpcBestCU->setPredictionMode(0, MODE_INTRA);
```

If next_calculation is False, then stop calling ```xCompressCU``` recursively. But directly assign:

```cpp
pcSubBestPartCU->getTotalCost() = MAX_DOUBLE / 16;
pcSubBestPartCU->getTotalDistortion() = MAX_UINT >> 3;
pcSubBestPartCU->getTotalBits() = MAX_UINT >> 3;
pcSubBestPartCU->setPartitionSize(0, SIZE_2Nx2N);
pcSubBestPartCU->setPredictionMode(0, MODE_INTRA);
```

## Reference
- [Fast CU Depth Decision Algorithm for HEVC Intra Coding -- CSDN](https://blog.csdn.net/beechina/article/details/25430737)
- [HEVC-Complexity-Reduction -- GitHub](https://github.com/tianyili2017/HEVC-Complexity-Reduction)
