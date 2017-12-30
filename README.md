# Vision-based Real-Time Aerial Object Localization and Tracking for UAV Sensing System

This repository contains the code (implemented in C++ on Linux) and dataset for our [IEEE-Access-2017-paper](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8080161). 

If you find our code or project useful in your research, please cite:

    @ARTICLE{Wu_Vision2017, 
    author={Y. Wu and Y. Sui and G. Wang}, 
    journal={IEEE Access}, 
    title={Vision-Based Real-Time Aerial Object Localization and Tracking for UAV Sensing System}, 
    year={2017}, 
    volume={5},
    pages={23969-23978},
    doi={10.1109/ACCESS.2017.2764419}
    }

Please also consider citing:

    @inproceedings{zhang2015minimum,
    title={Minimum barrier salient object detection at 80 fps},
    author={Zhang, Jianming and Sclaroff, Stan and Lin, Zhe and Shen, Xiaohui and 
    Price, Brian and Mech, Radomir},
    booktitle={Proceedings of the IEEE International Conference on Computer Vision},
    pages={1404--1412},
    year={2015}
    }

## Prerequisite

OpenCV 2.4+

Cmake 2.8

## Usage
1. Download our dataset and untar to the correct path (../../../dataset/), or you 
can use your own dataset.
2. go to /path/to/code/MBSPlusKF/, mkdir build
3. cd build
4. cmake ..
5. make 
6. ./MBSPlusKF ../../../dataset/ 

(7. If want to save results: ./MBSPlusKF ../../../dataset/ ../../../Tracking_results/airplane_016/)
