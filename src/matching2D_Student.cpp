#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorFamily, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = descriptorFamily.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
        // cout << "BF matching cross-check=" << crossCheck;
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        if (descSource.type() != CV_32F)
        { // OpenCV bug workaround : convert binary descriptors to floating point due to a bug in current OpenCV implementation
            descSource.convertTo(descSource, CV_32F);
            descRef.convertTo(descRef, CV_32F);
        }

        //implement FLANN matching
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
        // cout << "FLANN matching";
    }


    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)
        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << " (NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

        vector<vector<cv::DMatch>> knnMatches;
        double t = (double)cv::getTickCount();

        // implement k-nearest-neighbor matching
        matcher->knnMatch(descSource, descRef, knnMatches, 2);
        // filter matches using descriptor distance ratio test
        const float ratio_thresh = 0.8f;
        
        for (auto knnMatch : knnMatches)
        {
            if (knnMatch[0].distance < ratio_thresh * knnMatch[1].distance)
                matches.push_back(knnMatch[0]);
        }
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        // cout << " (KNN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;
        // cout << "# keypoints removed = " << knnMatches.size() - matches.size() << endl;

    }
}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
    else if (descriptorType.compare("BRIEF") == 0)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create(); // needs xfeatures2d
    }
    else if (descriptorType.compare("ORB") == 0)
    {
        extractor = cv::ORB::create();
    }
    else if (descriptorType.compare("FREAK") == 0)
    {
        extractor = cv::xfeatures2d::FREAK::create(); // needs xfeatures2d
    }
    else if (descriptorType.compare("AKAZE") == 0)
    {
        extractor = cv::AKAZE::create();
    }
    else if (descriptorType.compare("SIFT") == 0)
    {
        extractor = cv::SIFT::create();
    }
    else 
    {
        throw std::invalid_argument(descriptorType + "is not a valid descriptor type");
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

     // add corners to result vector
    for (auto point : corners)
    {
        cv::KeyPoint newKeyPoint(point, blockSize); // initialize point use cv::Point2f point and size
        keypoints.push_back(newKeyPoint);
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using the Harris corner detector
void detKeypointsHarris(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered (Harris 2x2 covariance matrix)
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 90; // minimum value for a corner in the 8bit scaled response matrix
    double k = 0.04;       // Harris parameter (see equation for details)

    // Apply corner detection
    double t = (double)cv::getTickCount();
    
    // Detect Harris corners and normalize output to 8 bit with convertScaleAbs
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    // Locate local maxima in the Harris response matrix 
    // and perform a non-maximum suppression (NMS) in a local neighborhood around 
    // each maximum. The resulting coordinates shall be stored in keypoints
    // Keypoint candidates have a higher response than the minimum

    float kpDiam = 2 * apertureSize; // diameter is twice the Sobel operator aperture
    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression
    for (int r=0; r<dst_norm.rows; r++)
    {
        for (int c=0; c<dst_norm.cols; c++)
        {
            float resp = dst_norm.at<float>(r,c);
            if (resp >= minResponse)
            {
                cv::KeyPoint kp(c, r, kpDiam, resp);

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool overlap = false;
                // Iterate through existing keypoints
                for (auto it = keypoints.begin(); it != keypoints.end(); it++)
                {   
                    // If there is an overlap with an existing keypoint
                    if (cv::KeyPoint::overlap(kp, *it) > maxOverlap)
                    {
                        overlap = true;
                        // If new keypoint has the higher response, replace old one and stop comparison
                        if (resp > it->response)
                        {
                            *it = kp;
                            break;
                        }
                    }
                }
                // If there was no overlap, push keypoint to vector
                if (!overlap)
                    keypoints.push_back(kp);
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Harris Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

// Detect keypoints in image using one of the modern detector types
void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
{
    cv::Ptr<cv::FeatureDetector> detector;
    if (detectorType.compare("FAST") == 0)
    {
        int threshold = 30;  // difference between intensity of the central pixel and pixels of a circle around this pixel
        bool bNMS = true;                   // perform non-maxima suppression on keypoints
        cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
        detector = cv::FastFeatureDetector::create(threshold, bNMS, type);;
    }
    else if (detectorType.compare("BRISK") == 0)
    {
        detector = cv::BRISK::create();
    }
    else if (detectorType.compare("ORB") == 0)
    {
        detector = cv::ORB::create();
    }
    else if (detectorType.compare("AKAZE") == 0)
    {
        detector = cv::AKAZE::create();
    }
    else if (detectorType.compare("SIFT") == 0)
    {
        detector = cv::SIFT::create();
    }
    else 
    {
        throw std::invalid_argument(detectorType + "is not a valid detector type");
    }
    
    double t = (double)cv::getTickCount();
    detector->detect(img, keypoints);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    // cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    
    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = detectorType + " Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}
