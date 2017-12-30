## Vision-based Real-Time Aerial Object Localization and Tracking for UAV Sensing System

[Yuanwei Wu](https://ryancv.github.io), [Yao Sui](http://www.suiyao.me/), and [Guanghui Wang](http://www.ittc.ku.edu/~ghwang/)

<img href="Post_processing.png", style="width:200px;height=100px">


### Abstract

The paper focuses on the problem of vision-based obstacle detection and tracking for unmanned aerial vehicle navigation. A real-time object localization and tracking strategy from monocular image sequences is developed by effectively integrating the object detection and tracking into a dynamic Kalman model. At the detection stage, the object of interest is automatically detected and localized from a saliency map computed via the image background connectivity cue at each frame; at the tracking stage, a Kalman filter is employed to provide a coarse prediction of the object state, which is further refined via a local detector incorporating the saliency map and the temporal information between two consecutive frames. Compared to existing methods, the proposed approach does not require any manual initialization for tracking, runs much faster than the state-of-the-art trackers of its kind, and achieves competitive tracking performance on a large number of image sequences. Extensive experiments demonstrate the effectiveness and superior performance of the proposed approach.


### Markdown

Markdown is a lightweight and easy-to-use syntax for styling your writing. It includes conventions for

```markdown
Syntax highlighted code block

# Header 1
## Header 2
### Header 3

- Bulleted
- List

1. Numbered
2. List

**Bold** and _Italic_ and `Code` text

[Link](url) and ![Image](src)
```

For more details see [GitHub Flavored Markdown](https://guides.github.com/features/mastering-markdown/).

### Jekyll Themes

Your Pages site will use the layout and styles from the Jekyll theme you have selected in your [repository settings](https://github.com/RyanCV/Vision-based-OLT/settings). The name of this theme is saved in the Jekyll `_config.yml` configuration file.

### Support or Contact

Having trouble with Pages? Check out our [documentation](https://help.github.com/categories/github-pages-basics/) or [contact support](https://github.com/contact) and we’ll help you sort it out.
