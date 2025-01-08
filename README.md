# AAGLSD
Aligned-Anchor Groups Guided Line Segment Detector 

Followings are brief description of each folder:
- src: the implementation of AAGLSD.
- pred_results: the .txt files of predicted line segments, each row is a line segment presented as ***[x0, y0, x1, y1]***.

## Requirements
- **C++17**
- **OpenCV >=4.6.0**

## Evaluation Results on the YorkUrban and the YorkUrban-LineSegment
![Fsc-YUD](./imgs/F-Score_YUD.png)  
![Fsc-YULD](./imgs/F-Score_YULD.png)  

## Prediction Results on the YorkUrban
![#1](./imgs/P1020829.jpg)  

![#2](./imgs/P1020838.jpg)  

![#3](./imgs/P1020860.jpg)
