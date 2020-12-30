/*

Tracker based on Kernelized Correlation Filter (KCF) [1] and Circulant Structure with Kernels (CSK) [2].
CSK is implemented by using raw gray level features, since it is a single-channel filter.
KCF is implemented by using HOG features (the default), since it extends CSK to multiple channels.

[1] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"High-Speed Tracking with Kernelized Correlation Filters", TPAMI 2015.

[2] J. F. Henriques, R. Caseiro, P. Martins, J. Batista,
"Exploiting the Circulant Structure of Tracking-by-detection with Kernels", ECCV 2012.

Default values are set for all properties of the tracker depending on the above choices.
Their values can be customized further before calling init():
    interp_factor: linear interpolation factor for adaptation
    sigma: gaussian kernel bandwidth
    lambda: regularization
    cell_size: HOG cell size
    padding: horizontal area surrounding the target, relative to its size
    output_sigma_factor: bandwidth of gaussian target
    template_size: template size in pixels, 0 to use ROI size
    scale_step: scale step for multi-scale estimation, 1 to disable it
    scale_weight: to downweight detection scores of other scales for added stability

For speed, the value (template_size/cell_size) should be a power of 2 or a product of small prime numbers.

Inputs to init():
   image is the initial frame.
   roi is a cv::Rect with the target positions in the initial frame

Inputs to update():
   image is the current frame.

Outputs of update():
   cv::Rect with target positions for the current frame

 */

#pragma once

#include <opencv2/opencv.hpp>
#include <string>

// abstract class
class TrackerTemp
{
public:
    TrackerTemp()  {}
   virtual  ~TrackerTemp() { }

    virtual void init(const cv::Rect &roi, cv::Mat image) = 0;
    virtual cv::Rect  update( cv::Mat image)=0;

protected:
    cv::Rect_<float> _roi;
};

#ifndef _OPENCV_CORETRACKER_HPP_
#define _OPENCV_CORETRACKER_HPP_
#endif

// core tracker
class CoreTracker : public TrackerTemp
{
public:
    // Constructor
    CoreTracker();

    // Initialize tracker 
    virtual void init(const cv::Rect &roi, cv::Mat image);
    
    // Update position based on the new frame
    virtual cv::Rect update(cv::Mat image);

    float interp_factor; // linear interpolation factor for adaptation
    float sigma; // gaussian kernel bandwidth
    float lambda; // regularization
    int cell_size; // HOG cell size
    int cell_sizeQ; // cell size^2, to avoid repeated operations
    float padding; // extra area surrounding the target
    float output_sigma_factor; // bandwidth of gaussian target
    int template_size; // template size
    float scale_step; // scale step for multi-scale estimation
    float scale_weight;  // to downweight detection scores of other scales for added stability

protected:
    // Detect object in the current frame.
    cv::Point2f detect(cv::Mat z, cv::Mat x, float &peak_value);

    // train tracker with a single image
    void train(cv::Mat x, float train_interp_factor);

    // Evaluates a Gaussian kernel with bandwidth SIGMA for all relative shifts between input images X and Y, which must both be MxN. They must    also be periodic (ie., pre-processed with a cosine window).
    cv::Mat gaussianCorrelation(cv::Mat x1, cv::Mat x2);

    // Create Gaussian Peak. Function called only in the first frame.
    cv::Mat createGaussianPeak(int sizey, int sizex);

    // Obtain sub-window from image, with replication-padding and extract features
    cv::Mat getFeatures(const cv::Mat & image, bool inithann, float scale_adjust = 1.0f);

    // Initialize Hanning window. Function called only in the first frame.
    void createHanningMats();

    // Calculate sub-pixel peak for one dimension
    float subPixelPeak(float left, float center, float right);

    cv::Mat _alphaf;
    cv::Mat _prob;
    cv::Mat _tmpl;
    cv::Mat _num;
    cv::Mat _den;
    cv::Mat _labCentroids;

private:
    int size_patch[3];
    cv::Mat hann;
    cv::Size _tmpl_sz;
    float _scale;
    int _gaussian_size;
    bool _hogfeatures;
    bool _labfeatures;
};
