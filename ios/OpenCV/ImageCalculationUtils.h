//
//  ImageCalculationUtils.hpp
//  RNDocumentScanner
//
//  Created by Iuri Cernov on 04/08/20.
//  Copyright © 2020 Facebook. All rights reserved.
//

#ifndef ImageCalculationUtils_hpp
#define ImageCalculationUtils_hpp

#import <opencv2/imgcodecs/ios.h>

typedef struct LineParameters {
    double rho;
    double theta;
} LineParameters;

LineParameters twoPointsToPolar(cv::Vec4i points);

void printLineParameters(LineParameters lp);
std::vector<LineParameters> filterSimilarLines(std::vector<LineParameters> lines, double rhoTolerance, double thetaTolerance);
std::vector<std::vector<LineParameters>> groupByNearParallelLines(std::vector<LineParameters> lines, double thetaTolerance);
std::vector<cv::Point> getIntersectionsFromParallelLines(std::vector<std::vector<LineParameters>> lineGroups);

#endif /* ImageCalculationUtils_hpp */
