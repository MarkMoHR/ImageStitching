# ImageStitching

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/singles.png)
![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/blendedImg0.png)


## 1. Requirements
- Windows10 + VS2015
- C++
- cimg library : http://www.cimg.eu/
- opencv (For extracting features of images)

---

## 2. Main Procedure
1. Image **feature extraction** with `SIFT` algorithm
1. Image feature points **matching** with `RANSAC` algorithm
1. Image **blending** with matched feature points

---

## 3. Intermediate Results

#### 1) Image feature extraction with `SIFT` algorithm
> relevant code: `MySift.h` and `MySift.cpp`
- results of key feature points (each with a feature descriptor of 128 dimention) of two images:

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/kps.png)

#### 2) Image feature points **matching** with `RANSAC` algorithm
> relevant code: `MyMatching.h` and `MyMatching.cpp`
- First do a *coarse-grained* feature points matching by calculating the distance of two feature descriptors, and regard the two points as matched if the distance is lower than some threshold. The matched points are lined together as shown below:

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/kps_real.png)

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/mixImgWithLine.png)

- Clearly there exist many outliers, which can be removed by `RANSAC` algorithm as shown below. The algorithm works on selecting the main transforming direction with most inliers:

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/ransac.png)

- Removed the outliers which are conflicted with the selected transforming direction:

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/mixImgWithLine_fixed.png)

#### 3) Image **blending** with matched feature points
> relevant code: `MyBlending.h` and `MyBlending.cpp`
- First use a simple translation method:

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/mixImg.png)

becomes

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/blended.png)

- Then apply a RGB interpolation at fusion region `A/B`:

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/blend.png)

- Stitched Result of two images

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/blendedImg.png)

- Repeat this procedure and get the stitched Result of all images

![Image text](https://github.com/MarkMoHR/ImageStitching/raw/master/figures/blendedImg0.png)

