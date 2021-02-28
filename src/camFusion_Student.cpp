
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"


#include "lidar/euclideanCluster.h"
#include "lidar/euclideanCluster.cpp"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0); 
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0); 

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}

/* 
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size. 
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // ...
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // ...
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // Cluster points in previous and current image
    // max distance
    double maxDist = 0.125;    
    std::vector<std::vector<LidarPoint>> prevClusters = euclideanCluster(lidarPointsPrev, maxDist, 10, 10000);
    std::vector<std::vector<LidarPoint>> currClusters = euclideanCluster(lidarPointsCurr, maxDist, 10, 10000);

    // Pick largest clusters for prev and current frame
    auto largePrevCluster = std::max_element(std::begin(prevClusters),
                                     std::end(prevClusters),
                                     [](const std::vector<LidarPoint>& lhs,
                                        const std::vector<LidarPoint>& rhs)
                                     {
                                       return lhs.size() < rhs.size();
                                     });
    auto largeCurrCluster = std::max_element(std::begin(currClusters),
                                     std::end(currClusters),
                                     [](const std::vector<LidarPoint>& lhs,
                                        const std::vector<LidarPoint>& rhs)
                                     {
                                       return lhs.size() < rhs.size();
                                     });

    // Pick closest points (along x) in current cluster
    // auto prevClosePt = std::min_element(largePrevCluster->begin(),
    //                                  largePrevCluster->end(),
    //                                  [](const LidarPoint& lhs,
    //                                     const LidarPoint& rhs)
    //                                  {
    //                                    return lhs.x < rhs.x;
    //                                  });
    auto currClosePt = std::min_element(largeCurrCluster->begin(),
                                     largeCurrCluster->end(),
                                     [](const LidarPoint& lhs,
                                        const LidarPoint& rhs)
                                     {
                                       return lhs.x < rhs.x;
                                     });

    // Pick median points (along x) in previous current cluster --> more robust for speed computation
    std::sort(largePrevCluster->begin(),
                largePrevCluster->end(),
                [](const LidarPoint& lhs,
                const LidarPoint& rhs)
                {
                return lhs.x < rhs.x;
                });
    LidarPoint prevMedianPt = largePrevCluster->at((int)largePrevCluster->size()/2);
    std::sort(largeCurrCluster->begin(),
                largeCurrCluster->end(),
                [](const LidarPoint& lhs,
                const LidarPoint& rhs)
                {
                return lhs.x < rhs.x;
                });
    LidarPoint currMedianPt = largeCurrCluster->at((int)largeCurrCluster->size()/2);

    // Compute TTC (speed computed with median points, distance with closest point)

    TTC = currClosePt->x /(frameRate*(prevMedianPt.x - currMedianPt.x));

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{   
    // For all boxes on previous frame
    for (auto prevBox : prevFrame.boundingBoxes)
    {   
        // Best box ID on current frame and best nr of matches
        int bestCurrId, bestMatches{0};
        // For all boxIDs on current image
        for (auto currBox: currFrame.boundingBoxes)
        {
            int nrMatches{0};
            // For all keypoints match
            for (auto match : matches)
            {
                // Extract matching keypoints in previous and current frame
                cv::KeyPoint prevKp = prevFrame.keypoints.at(match.queryIdx);
                cv::KeyPoint currKp = currFrame.keypoints.at(match.trainIdx);

                // If previous box contains previous keypoint and current box contains current keypoint, increase number of matches
                if (prevBox.roi.contains(prevKp.pt) && currBox.roi.contains(currKp.pt))
                    nrMatches++;
            } // eof loop over all keypoint matches

            // If nr matches is above best match, make best match
            if (nrMatches > bestMatches)
            {
                bestMatches = nrMatches;
                bestCurrId = currBox.boxID;
            }

        } // eof loop over all current box IDs

        // Add to bbBestMatches if number of matches is at least 1
        if (bestMatches > 0)
        {
            bbBestMatches.insert(std::make_pair(prevBox.boxID, bestCurrId));
        }

    } // eof loop over all previous box IDs
}

// void matchBoundingBoxesOld(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
// {
//     std::map<std::pair<int, int>, int> bbNrMatches; // map containing as key pairs of bounding box IDs and as value number of keypoint matches
//     for (auto match : matches)
//     {
//         // Find keypoints from match in previous and current frame
//         cv::KeyPoint prevKp = prevFrame.keypoints.at(match.queryIdx);
//         cv::KeyPoint currKp = currFrame.keypoints.at(match.trainIdx);

//         vector<int> prevIds, currIds; // IDs of boxes containing keypoints in prev and current image

//         // Collect box IDs containing previous keypoint
//         for (auto it = prevFrame.boundingBoxes.begin(); it != prevFrame.boundingBoxes.end(); it++)
//         {
//             if (it->roi.contains(prevKp.pt)
//                 prevIds.push_back(it->boxID);
//         }

//         // Collect box IDs containing current keypoint
//         for (auto it = currFrame.boundingBoxes.begin(); it != currFrame.boundingBoxes.end(); it++)
//         {
//             if (it->roi.contains(currKp.pt)
//                 currIds.push_back(it->boxID);
//         }

//         // If at least a box was found on each frame, add all box pairs to bbMatches
//         if (!(prevIds.empty()) && !(currIds.empty()) 
//         {
//             for (auto prevId : prevIds)
//             {
//                 for (auto currId : currIds)
//                 {
//                     std::pair<int, int> bbMatch = make_pair(prevId, currId);
//                     // Find if bbMatch is present in map
//                     auto map_it = bbNrMatches.find(bbMatch);
//                     // if iterator is the end of the map, bbMatch was not found --> initialize at 1
//                     if (map_it == bbNrMatches.end())
//                         bbNrMatches[bbMatch] = 1;
//                     // otherwise, increment by 1
//                     else
//                         bbNrMatches[bbMatch] = map_it->second + 1;
//                 }
//             }
//         }

//     } // eof loop over all keypoint matches
    
//     // For all boxIDs on previous image
//     for (auto prevId : prevFrame.boundingBoxes.boxID)
//     {
//         std::pair<int, int> b = make_pair(-1, -1);
//         // Alt 1 -> bbMatches pairs vector, count pairs where first element is boxID and pick the one with highest nr of matches
//         // Alt 2 -> bbNrMatches map pairs -> int, pick pair with first key boxID with highest nr of matches
//     }
// }
