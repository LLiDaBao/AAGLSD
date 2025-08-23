# 😎ALIGNED ANCHOR GROUPS GUIDED LINE SEGMENT DETECTOR
## 😊Introduction
The implementation of paper **"ALIGNED ANCHOR GROUPS GUIDED LINE SEGMENT DETECTOR"**. 

Followings are brief description of each folder:
```
AAGLSD/
├── evaluation:   Code for evaluation. 
├── imgs:         Images for readme.
├── pred_results: Results from different line segment detectors. 
├── src/
│    ├── src_cpp:  C++ implementation of AAGLSD.
│    └── src_py:   Python implementation of AAGLSD.
└── README.md
```
p.s.
- evaluation: Only `repeatbility` evaluation is available. For the other metrics we adopted, the code is from [AG3Line](https://github.com/weidong-whu/AG3line) now. The python code would be assembled soon.
- pred_results: Detected line segments from different line segment detectors. Each row a `.txt` file is a line segment represented as endpoint-endpoint`[x0, y0, x1, y1]`, which means endpoint-endpoint. The suffix represents the type of LSD (e.g. `*aag.txt`).

## 🚀 TODO
- [x] Date: 2025.06.30.
  - Release C++ source code of our AAGLSD.
  - Release `repeatability` evaluation code.

- [x] Date: 2025.8.23.
  - 🥰Our paper is accepted by **PRCV2025** (The 8th Chinese Conference on Pattern Recognition and Computer Vision).
- [ ] Release python code for AAGLSD and metrics evaluation (AP,AR,IoU,F-Score).



## 📖 Evaluation Results on the YorkUrbanDB
The F-Score for different line segment detectors evaluated on the [YorkUrbanDB](https://www.elderlab.yorku.ca/resources/york-urban-line-segment-database-information/), and the YorkUrban-LineSegment from [Linelet](https://github.com/NamgyuCho/Linelet-code-and-YorkUrban-LineSegment-DB).
### 📗YorkUrbanDB
<div align="center">
    <img src="./imgs/AP-yorkurban.png" width="30%" alt="AP YUD"/>
    <img src="./imgs/AR-yorkurban.png" width="30%" alt="AR YUD"/>
    <img src="./imgs/Fsc-yorkurban.png" width="30%" alt="F-Score YUD"/>
</div>

### 📗YorkUrban-LineSegment
<div align="center">
    <img src="./imgs/AP-yorkurban.png" width="30%" alt="AP YULD"/>
    <img src="./imgs/AR-yorkurban.png" width="30%" alt="AR YULD"/>
    <img src="./imgs/Fsc-yorkurban.png" width="30%" alt="F-Score YULD"/>
</div>

## ⏰ Visulization
From left to right: 

- *Aligned Anchor Groups*
- *Regular Anchors*
- *Detected Line Segments*
### YorkUrban Visualization
<div align="center">
    <img src="./imgs/P1020829-AAG_SHOW.png" width="30%" /> 
    <img src="./imgs/P1020829-RA_SHOW.png" width="30%" /> 
    <img src="./imgs/P1020829-RES.png" width="30%" /> 
</div> 

<div align="center">
    <img src="./imgs/P1040795-AAG_SHOW.png" width="30%" /> 
    <img src="./imgs/P1040795-RA_SHOW.png" width="30%" /> 
    <img src="./imgs/P1040795-RES.png" width="30%" /> 
</div> 

### HPatches Visualization
We also evaluated on the illumination subset of [HPathces](https://github.com/hpatches/hpatches-dataset). Here is the example of `castle` and `book` sequences.
<div align="center">
    <img src="./imgs/castle1.jpg" width="30%" /> 
    <img src="./imgs/castle2.jpg" width="30%" /> 
    <img src="./imgs/castle3.jpg" width="30%" /> 
</div> 

<div align="center">
    <img src="./imgs/castle4.jpg" width="30%" /> 
    <img src="./imgs/castle5.jpg" width="30%" /> 
    <img src="./imgs/castle6.jpg" width="30%" /> 
</div> 

<div align="center">
    <img src="./imgs/book1.jpg" width="30%" /> 
    <img src="./imgs/book2.jpg" width="30%" /> 
    <img src="./imgs/book3.jpg" width="30%" /> 
</div> 

<div align="center">
    <img src="./imgs/book4.jpg" width="30%" /> 
    <img src="./imgs/book5.jpg" width="30%" /> 
    <img src="./imgs/book6.jpg" width="30%" /> 
</div
