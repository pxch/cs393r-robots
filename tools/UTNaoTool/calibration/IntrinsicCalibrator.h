#ifndef CHECKERBOARD_CALIBRATOR_H
#define CHECKERBOARD_CALIBRATOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <time.h>
#include <map>


struct ICMeasures {
  float x, y, size, skew;
};

typedef ICMeasures ICProgress;

class IntrinsicCalibrator {
  public:
    enum Pattern {
      CHESSBOARD = 0,
      CIRCLES_GRID = 1,
      ASYMMETRIC_CIRCLES_GRID = 2
    };
    struct ICSettings {
      cv::Size imageSize;
      cv::Size boardSize;
      Pattern patternType;
      float squareSize;
      float aspectRatio;
      unsigned int flags;
      ICSettings();
    };
  private: 
    std::vector<std::vector<cv::Point2f> > imagePoints_;
    std::vector<std::vector<cv::Point3f> > objectPoints_;
    std::vector<ICMeasures> sampleParams_;
    cv::Mat cameraMatrix_, distortionCoeffs_;
    ICSettings settings_;
    std::vector<cv::Mat> images_;

    double computeReprojectionErrors(
      const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
      std::vector<float>& perViewErrors) const;

    std::vector<cv::Point3f> calcChessboardCorners() const;
    
    bool runCalibration( 
      std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
      std::vector<float>& reprojErrs,
      double& totalAvgErr);

    void saveCameraParams(const std::string& filename,
       const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
       const std::vector<float>& reprojErrs,
       double totalAvgErr, bool writePoints) const;


    ICMeasures getParameters(std::vector<cv::Point2f> corners);
    bool isGoodSample(ICMeasures params);
  public:
    int getSampleCount();
    ICProgress getProgress();
    IntrinsicCalibrator(ICSettings settings = ICSettings());
    void addImages(const std::vector<cv::Mat>& images);
    std::vector<cv::Point2f> addImage(const cv::Mat& image);
    int sampleCount() const { return imagePoints_.size(); }
    bool runAndSave(const std::string& outputFilename, bool writeExtrinsics = false, bool writePoints = false);
};

#endif
