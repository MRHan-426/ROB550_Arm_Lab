#  Robot Arm v.s. Various Blocks

<p align="center">
  <img src="https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/stacking.gif" alt="gif">
</p>
     
![Arm Lab](https://img.shields.io/badge/ROB550-ArmLab-orange)
![Primary language](https://img.shields.io/github/languages/top/MRHan-426/ROB550_Arm_Lab)
![License](https://img.shields.io/badge/license-MIT-green)

This is Section2 Team 9's arm lab project git repository for ROB550: Robotic Sys Lab. 

The team members include: Ziqi(Jackie) Han, Siyuan(Lynx) Yin, Hongjiao(Oliver) Qiang.

## 1. Block Detection

Our implemented block detector exhibits robustness in the detection of the position, color, orientation of many kinds of blocks, including cuboid, arch, triangle, semi-circle, cylinder. The procedural steps of our algorithm primarily encompass initial object detection, Contour optimization via clustering, color recognition, and object classification.

+ Contour optimization via clustering
<p align="center">
  <img src="https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/cluster_1.png" alt="image" width="66%" height="auto" />
</p>
<p align="center">
  <img src="https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/cluster_4.png" alt="image" width="66%" height="auto" />
</p>

+ Color recognition
<p align="center">
  <img src="https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/color_detection.png" alt="image" width="66%" height="auto" />
</p>

+ Object classification
<p align="center">
  <img src="https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/edges_detect.png" alt="image" width="66%" height="auto" />
</p>

+ Result
<p align="center">
  <img src="https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/block_detect_result.png" alt="image" width="66%" height="auto" />
</p>

## 2. Robot Control
<p align="center">
  <img src="https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/robot_control.png" alt="image" width="66%" height="auto" />
</p>

## 3. Documentation

+ Project Document: [**[PDF]**](https://github.com/MRHan-426/ROB550_Arm_Lab/blob/main/doc/ROB_550_Report___Section_2___Group_9.pdf)

