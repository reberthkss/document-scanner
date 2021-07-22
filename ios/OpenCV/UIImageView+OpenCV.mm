//
//  UIImage+OpenCV.mm
//  OpenCVClient
//
//  Created by Washe on 01/12/2012.
//  Copyright 2012 Washe / Foundry. All rights reserved.
//
//  Permission is given to use this source code file without charge in any
//  project, commercial or otherwise, entirely at your risk, with the condition
//  that any redistribution (in part or whole) of source code must retain
//  this copyright and permission notice. Attribution in compiled projects is
//  appreciated but not required.
//
//  adapted from
//  http://docs.opencv.org/doc/tutorials/ios/image_manipulation/image_manipulation.html#opencviosimagemanipulation

#import "UIImage+OpenCV.h"
#import "UIImage+OpenCVBW.h"
#import "UIImage+Rotate.h"
#import "UIImageView+ContentFrame.h"
#import "UIImageView+OpenCV.h"
#import "ImageCalculationUtils.h"
#import <opencv2/imgcodecs/ios.h>

@implementation UIImageView (OpenCV)

bool detectBySides = true;

- (CropRect)detectEdges {
    cv::Mat original = self.image.CVMat;
    
    CGSize targetSize = self.contentSize;
    cv::resize(original, original, cvSize(targetSize.width, targetSize.height));
    
    std::vector<std::vector<cv::Point>>squares;
    std::vector<cv::Point> largest_square;
    
    CGFloat defaultWidth = targetSize.width * 0.5;
    CGFloat defaultHeight = defaultWidth;
    CGFloat defaultX = (targetSize.width - defaultWidth) / 2;
    CGFloat defaultY = (targetSize.height - defaultHeight) / 2;
    
    CropRect rect;
    rect.topLeft = CGPointMake(defaultX, defaultY);
    rect.topRight = CGPointMake(defaultX + defaultWidth, defaultY);
    rect.bottomLeft = CGPointMake(defaultX, defaultY + defaultHeight);
    rect.bottomRight = CGPointMake(defaultX + defaultWidth, defaultY + defaultHeight);
    
    find_squares(original, squares);

    // Adding inverted colors result
    cv::Mat inverted = self.image.CVMat;
    cv::resize(inverted, inverted, cvSize(targetSize.width, targetSize.height));
    cv::bitwise_not(inverted, inverted);
    find_squares(inverted, squares);

    find_largest_square(squares, largest_square);
    
    if (largest_square.size() == 4) {
        // Manually sorting points, needs major improvement. Sorry.
        
        NSMutableArray *points = [NSMutableArray array];
        NSMutableDictionary *sortedPoints = [NSMutableDictionary dictionary];
        
        for (int i = 0; i < 4; i++) {
            NSDictionary *dict = [NSDictionary dictionaryWithObjectsAndKeys:[NSValue valueWithCGPoint:CGPointMake(largest_square[i].x, largest_square[i].y)], @"point" , [NSNumber numberWithInt:(largest_square[i].x + largest_square[i].y)], @"value", nil];
            [points addObject:dict];
        }
        
        int min = [[points valueForKeyPath:@"@min.value"] intValue];
        int max = [[points valueForKeyPath:@"@max.value"] intValue];
        
        int minIndex = 0;
        int maxIndex = 0;
        
        int missingIndexOne = 0;
        int missingIndexTwo = 0;
        
        for (int i = 0; i < 4; i++) {
            NSDictionary *dict = [points objectAtIndex:i];
            
            if ([[dict objectForKey:@"value"] intValue] == min) {
                [sortedPoints setObject:[dict objectForKey:@"point"] forKey:@"0"];
                minIndex = i;
                continue;
            }
            
            if ([[dict objectForKey:@"value"] intValue] == max) {
                [sortedPoints setObject:[dict objectForKey:@"point"] forKey:@"2"];
                maxIndex = i;
                continue;
            }
            
            missingIndexOne = i;
        }
        
        for (int i = 0; i < 4; i++) {
            if (missingIndexOne != i && minIndex != i && maxIndex != i) {
                missingIndexTwo = i;
            }
        }
        
        if (largest_square[missingIndexOne].x < largest_square[missingIndexTwo].x) {
            //2nd Point Found
            [sortedPoints setObject:[[points objectAtIndex:missingIndexOne] objectForKey:@"point"] forKey:@"3"];
            [sortedPoints setObject:[[points objectAtIndex:missingIndexTwo] objectForKey:@"point"] forKey:@"1"];
        } else {
            //4rd Point Found
            [sortedPoints setObject:[[points objectAtIndex:missingIndexOne] objectForKey:@"point"] forKey:@"1"];
            [sortedPoints setObject:[[points objectAtIndex:missingIndexTwo] objectForKey:@"point"] forKey:@"3"];
        }
        
        rect.topLeft = [(NSValue *)[sortedPoints objectForKey:@"0"] CGPointValue];
        rect.topRight = [(NSValue *)[sortedPoints objectForKey:@"1"] CGPointValue];
        rect.bottomLeft = [(NSValue *)[sortedPoints objectForKey:@"3"] CGPointValue];
        rect.bottomRight = [(NSValue *)[sortedPoints objectForKey:@"2"] CGPointValue];
    }
    
    original.release();
    
    return rect;
}

std::vector<cv::Point> hull2Points(std::vector<int> hullIndices, std::vector<cv::Point> contour) {
    std::vector<cv::Point> points;
    for (int i = 0; i < hullIndices.size(); i++) {
        int index = hullIndices[i];
        points.push_back(contour[index]);
    }
    return points;
}

bool compareArea(const std::vector<cv::Point> v1, const std::vector<cv::Point> v2) {
    double a1 = cv::contourArea(v1);
    double a2 = cv::contourArea(v2);
    return a1 < a2;
}

void find_squares(cv::Mat& image, std::vector<std::vector<cv::Point>>&squares) {
    using namespace std;
    
    cv::Mat result(image);
    cv::cvtColor(image, result, CV_BGR2GRAY);

    // step 1
    cv::blur(result, result, cv::Size(3, 3));
    cv::normalize(result, result, 0, 255, cv::NORM_MINMAX);
    
    // step 2
    cv::threshold(result, result, 150, 255, cv::THRESH_TRUNC);
    cv::normalize(result, result, 0, 255, cv::NORM_MINMAX);
    
    // step 3
    cv::Canny(result, result, 185, 85);
    
    // step 4
    cv::threshold(result, result, 155, 255, cv::THRESH_TOZERO);

    // step 5
    cv::Mat morph_kernel = cv::Mat(cv::Size(10, 10), CV_8UC1, cv::Scalar(255));
    cv::morphologyEx(result, result, cv::MORPH_CLOSE, morph_kernel, cv::Point(-1, -1), 1);
    
    // get 10 largest contours:
    cv::Mat hierarchy = cv::Mat();
    vector<vector<cv::Point>> contourList = vector<vector<cv::Point>>();
    cv::findContours(result, contourList, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_SIMPLE);
    
    vector<vector<cv::Point>> hullList = vector<vector<cv::Point>>();
    vector<int> tempHullIndices = vector<int>();
    for (int i = 0; i < contourList.size(); i++) {
        cv::convexHull(contourList[i], tempHullIndices);
        hullList.push_back(hull2Points(tempHullIndices, contourList[i]));
    }
    
    sort(hullList.begin(), hullList.end(), compareArea);
    const int max = 40;
    int maxIndex = hullList.size() < max ? hullList.size() : max;
    for (int i = 0; i < maxIndex; i++) {
        vector<cv::Point> approx;
        approx = detectBySidesFromContour(cv::Mat(hullList[i]), image.size());
        squares.push_back(approx);
    }
}

void find_largest_square(const std::vector<std::vector<cv::Point> >& squares, std::vector<cv::Point>& biggest_square) {
    if (!squares.size()) {
        // no squares detected
        return;
    }
    
    int max_width = 0;
    int max_height = 0;
    int max_square_idx = 0;
    
    for (size_t i = 0; i < squares.size(); i++) {
        if (squares[i].size() != 4) continue;
        // Convert a set of 4 unordered Points into a meaningful cv::Rect structure.
        cv::Rect rectangle = boundingRect(cv::Mat(squares[i]));
        
        //        cout << "find_largest_square: #" << i << " rectangle x:" << rectangle.x << " y:" << rectangle.y << " " << rectangle.width << "x" << rectangle.height << endl;
        
        // Store the index position of the biggest square found
        if ((rectangle.width >= max_width) && (rectangle.height >= max_height)) {
            max_width = rectangle.width;
            max_height = rectangle.height;
            max_square_idx = i;
        }
    }
    
    biggest_square = squares[max_square_idx];
}

double angle( cv::Point pt1, cv::Point pt2, cv::Point pt0 ) {
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

- (CGPoint)coordinatesForPoint: (CGPoint)point withScaleFactor: (CGFloat)scaleFactor {
    return CGPointMake((point.x) / scaleFactor, (point.y) / scaleFactor);
}

- (UIImage *)crop: (CropRect)cropRect andApplyBW:(BOOL)applyBW {
    CGFloat scaleFactor =  [self contentScale];
    CGPoint ptBottomLeft = [self coordinatesForPoint:cropRect.bottomLeft withScaleFactor:scaleFactor];
    CGPoint ptBottomRight = [self coordinatesForPoint:cropRect.bottomRight withScaleFactor:scaleFactor];
    CGPoint ptTopRight = [self coordinatesForPoint:cropRect.topRight withScaleFactor:scaleFactor];
    CGPoint ptTopLeft = [self coordinatesForPoint:cropRect.topLeft withScaleFactor:scaleFactor];
    
    CGFloat w1 = sqrt( pow(ptBottomRight.x - ptBottomLeft.x , 2) + pow(ptBottomRight.x - ptBottomLeft.x, 2));
    CGFloat w2 = sqrt( pow(ptTopRight.x - ptTopLeft.x , 2) + pow(ptTopRight.x - ptTopLeft.x, 2));
    
    CGFloat h1 = sqrt( pow(ptTopRight.y - ptBottomRight.y , 2) + pow(ptTopRight.y - ptBottomRight.y, 2));
    CGFloat h2 = sqrt( pow(ptTopLeft.y - ptBottomLeft.y , 2) + pow(ptTopLeft.y - ptBottomLeft.y, 2));
    
    CGFloat maxWidth = (w1 < w2) ? w1 : w2;
    CGFloat maxHeight = (h1 < h2) ? h1 : h2;
    
    cv::Point2f src[4], dst[4];
    src[0].x = ptTopLeft.x;
    src[0].y = ptTopLeft.y;
    src[1].x = ptTopRight.x;
    src[1].y = ptTopRight.y;
    src[2].x = ptBottomRight.x;
    src[2].y = ptBottomRight.y;
    src[3].x = ptBottomLeft.x;
    src[3].y = ptBottomLeft.y;
    
    dst[0].x = 0;
    dst[0].y = 0;
    dst[1].x = maxWidth - 1;
    dst[1].y = 0;
    dst[2].x = maxWidth - 1;
    dst[2].y = maxHeight - 1;
    dst[3].x = 0;
    dst[3].y = maxHeight - 1;
    
    cv::Mat undistorted = cv::Mat( cvSize(maxWidth,maxHeight), CV_8UC4);
    cv::Mat original = self.image.CVMat;
    
    cv::warpPerspective(original, undistorted, cv::getPerspectiveTransform(src, dst), cvSize(maxWidth, maxHeight));
    
    UIImage *processedImage = [UIImage imageWithCVMat:undistorted];
    
    if (applyBW) {
        processedImage = [processedImage blackAndWhite];
    }
    
    for (UIView *subview in self.subviews) {
        [subview removeFromSuperview];
    }
    
    self.image = processedImage;
    
    return processedImage;
}


// detection by sides

int i = 0;

std::vector<cv::Point> detectBySidesFromContour(cv::Mat curve, cv::Size imageSize) {
    i++;
    printf("detectBySidesFromContour #%d\n", i);
    cv::Mat pictureOut = cv::Mat::zeros(imageSize, CV_8U);
    std::vector<cv::Mat> l;
    l.push_back(curve);
    cv::Scalar s = cv::Scalar(200, 200, 200, 200);
    cv::drawContours(pictureOut, l, -1, s);
    
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(pictureOut, lines, 1, M_PI / 180, 50, 10, 5.0);
    
    std::vector<LineParameters> linesParameters;
    for (int i = 0; i < lines.size(); i++) {
        linesParameters.push_back(twoPointsToPolar(lines[i]));
    }
    
    std::vector<LineParameters> similarLines = filterSimilarLines(linesParameters, 30, M_PI / 180 * 10);
    printf("Lines detected: %lu\n", similarLines.size());
    for (int i = 0; i < similarLines.size(); i++) {
        printf("Similar line: ");
        printLineParameters(similarLines[i]);
        linesParameters.push_back(twoPointsToPolar(lines[i]));
    }

    if (similarLines.size() == 4) {
        // We have 4 similar lines. It can be a rectangle
        std::vector<std::vector<LineParameters>> groupedByParallel = groupByNearParallelLines(similarLines, M_PI/180*20);
        if (groupedByParallel.size() == 2 && groupedByParallel[0].size() == 2 && groupedByParallel[1].size() == 2) {
            // Looks like we have pairs of near parallel lines, so bigger chance is a rectangle
            std::vector<cv::Point> intersections = getIntersectionsFromParallelLines(groupedByParallel);
            printf("Intersections: ");
            for (int i = 0; i < intersections.size(); i++) {
                printf("(%d,%d) ", intersections[i].x, intersections[i].y);
            }
            printf("\n");
            return intersections;
        }
    }
    std::vector<cv::Point> empty;
    return empty;
}

@end
