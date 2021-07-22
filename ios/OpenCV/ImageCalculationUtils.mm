//
//  ImageCalculationUtils.cpp
//  RNDocumentScanner
//
//  Created by Iuri Cernov on 04/08/20.
//  Copyright © 2020 Facebook. All rights reserved.
//

#include "ImageCalculationUtils.h"
#import <opencv2/imgcodecs/ios.h>
#import <cmath>

double normalizeAngle(double angle) {
    double newAngle = angle;
    while (newAngle <= -M_PI) newAngle += M_PI * 2;
    while (newAngle > M_PI) newAngle -= M_PI * 2;
    return newAngle;
}

LineParameters twoPointsToPolar(cv::Vec4i points) {
    int p1x = points[0];
    int p1y = points[1];
    int p2x = points[2];
    int p2y = points[3];
    LineParameters lp;
    
    if (p2x == p1x) {
        lp.rho = p2x; lp.theta = 0;
        return lp;
    }
    
    double m = ((double)p2y - (double)p1y) / ((double)p2x - (double)p1x);
    if (m < 0) {
        lp.theta = normalizeAngle(M_PI/2 - atan(-m));
        lp.rho = abs(-(double)p1y + m*p1x) / sqrt(m*m+1);
        return lp;
    } else if (m > 0) {
        double c = p1y - m*p1x;
        if (c < 0) {
            lp.theta = normalizeAngle(M_PI/2 + atan(m));
            lp.rho = -abs(-(double)p1y + m*p1x) / sqrt(m*m+1);
            return lp;
        } else {
            lp.theta = normalizeAngle(M_PI/2 + atan(m));
            lp.rho = abs(-(double)p1y + m*p1x) / sqrt(m*m+1);
            return lp;
        }
    }
    lp.theta = M_PI/2;
    lp.rho = abs(-(double)p1y + m*p1x) / sqrt(m*m+1);
    return lp;
}

void printLineParameters(LineParameters lp) {
    printf("theta = %fdeg, rho = %f\n", lp.theta*180.0/M_PI, lp.rho);
}

bool linesHasCloseAbsoluteParameters(LineParameters l1, LineParameters l2, double rhoTolerance, double thetaTolerance) {
    // 1 - Rhos are too much different - not similar
    if (abs(l1.rho - l2.rho) > rhoTolerance) return false;
    
    // 2 - Thetas are close, they are similar
    return abs(l1.theta - l2.theta) <= thetaTolerance;
}

LineParameters transformCoordinates(LineParameters line) {
    if (line.theta > M_PI / 2) {
        LineParameters lp;
        lp.theta = line.theta - M_PI;
        lp.rho = -line.rho;
        return lp;
    } else {
        return line;
    }
}

bool areLinesSimilar(LineParameters l1, LineParameters l2, double rhoTolerance, double thetaTolerance) {
    if (linesHasCloseAbsoluteParameters(l1, l2, rhoTolerance, thetaTolerance)) return true;
    // Maybe some theta is different. Let's try moving some coordinates
    LineParameters l1t = transformCoordinates(l1);
    LineParameters l2t = transformCoordinates(l2);
    return linesHasCloseAbsoluteParameters(l1t, l2t, rhoTolerance, thetaTolerance);
}

std::vector<LineParameters> filterSimilarLines(std::vector<LineParameters> lines, double rhoTolerance, double thetaTolerance) {
    using namespace std;
    vector<vector<LineParameters>> chunks;
    for (int lineIndex = 0; lineIndex < lines.size(); lineIndex++) {
        LineParameters line = lines[lineIndex];
        bool addedToAChunk = false;
        for (int chunkIndex = 0; chunkIndex < chunks.size(); chunkIndex++) {
            LineParameters firstLineFromChunk = chunks[chunkIndex][0];
            if (areLinesSimilar(line, firstLineFromChunk, rhoTolerance, thetaTolerance)) {
                chunks[chunkIndex].push_back(line);
                addedToAChunk = true;
                break;
            }
        }
        if (!addedToAChunk) {
            vector<LineParameters> chunk;
            chunk.push_back(line);
            chunks.push_back(chunk);
        }
    }
    vector<LineParameters> result;
    for (int chunkIndex = 0; chunkIndex < chunks.size(); chunkIndex++) {
        // No average calculation - just one parameter of chunk
        result.push_back(chunks[chunkIndex][0]);
    }
    return result;
}

bool areThetasClose(double firstTheta, double secondTheta, double thetaTolerance) {
    // 1 - Thetas are close, they are similar
    if (abs(firstTheta - secondTheta) <= thetaTolerance) return true;

    // 2 - Maybe thetas are different like 179 deg and 0 deg
    if (abs(abs(firstTheta - secondTheta) - M_PI) <= thetaTolerance) return true;

    // 3 - They are not similar
    return false;
}

std::vector<std::vector<LineParameters>> groupByNearParallelLines(std::vector<LineParameters> lines, double thetaTolerance) {
    using namespace std;
    vector<vector<LineParameters>> dividedLines;
    for (int i = 0; i < lines.size(); i++) {
        LineParameters line = lines[i];
        bool addedToAChunk = false;
        for (int chunkIndex = 0; chunkIndex < dividedLines.size(); chunkIndex++) {
            LineParameters firstDividedLineFromChunk = dividedLines[chunkIndex][0];
            if (areThetasClose(line.theta, firstDividedLineFromChunk.theta, thetaTolerance)) {
                dividedLines[chunkIndex].push_back(line);
                addedToAChunk = true;
                break;
            }
        }
        if (!addedToAChunk) {
            vector<LineParameters> chunk;
            chunk.push_back(line);
            dividedLines.push_back(chunk);
        }
    }
    return dividedLines;
}

std::vector<cv::Point2f> getTwoPointsOfLine(LineParameters line) {
    double x1 = cos(line.theta)*line.rho;
    double y1 = sin(line.theta)*line.rho;
    double theta2 = M_PI/2 - line.theta;
    double deltaX = 100.0*cos(theta2);
    double deltaY = -100.0*sin(theta2);
    double x2 = x1 + deltaX;
    double y2 = y1 + deltaY;
    std::vector<cv::Point2f> result;
    result.push_back(cv::Point2f(x1,y1));
    result.push_back(cv::Point2f(x2,y2));
    return result;
}

cv::Point getIntersection(LineParameters l1, LineParameters l2) {
    std::vector<cv::Point2f> pointsFromL1 = getTwoPointsOfLine(l1);
    std::vector<cv::Point2f> pointsFromL2 = getTwoPointsOfLine(l2);
    cv::Point2f p1 = pointsFromL1[0];
    cv::Point2f p2 = pointsFromL1[1];
    cv::Point2f p3 = pointsFromL2[0];
    cv::Point2f p4 = pointsFromL2[1];
    // https://en.wikipedia.org/wiki/Line–line_intersection
    double t = ((p1.x-p3.x)*(p3.y-p4.y) - (p1.y-p3.y)*(p3.x-p4.x))/((p1.x-p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));
    double x = p1.x + t*(p2.x - p1.x);
    double y = p1.y + t*(p2.y - p1.y);
    return cv::Point(x, y);
}

std::vector<cv::Point> getIntersectionsFromParallelLines(std::vector<std::vector<LineParameters>> lineGroups) {
    std::vector<cv::Point> result;
    result.push_back(getIntersection(lineGroups[0][0], lineGroups[1][1]));
    result.push_back(getIntersection(lineGroups[0][0], lineGroups[1][0]));
    result.push_back(getIntersection(lineGroups[0][1], lineGroups[1][0]));
    result.push_back(getIntersection(lineGroups[0][1], lineGroups[1][1]));
    return result;
}
