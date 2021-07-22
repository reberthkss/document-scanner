package com.ubidreams.RNDocumentScanner;

import org.opencv.core.Point;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class ImageCalculationUtils {
    // https://math.stackexchange.com/questions/1796400/estimate-line-in-theta-rho-space-given-2-points
    static LineParameters twoPointsToPolar(Point p1, Point p2) {
        if (p2.x == p1.x) { // Vertical line
            final double x = p2.x;
            return new LineParameters(x, 0);
        }
        double m = (p2.y - p1.y) / (p2.x - p1.x);
        if (m < 0) {
            double theta = normalizeAngle(Math.PI/2 - Math.atan(-m));
            double rho = Math.abs(-p1.y + m*p1.x) / Math.sqrt(Math.pow(m, 2) + 1);
            return new LineParameters(rho, theta);
        } else if (m > 0) {
            double c = p1.y - m*p1.x;
            if (c < 0) {
                double theta = normalizeAngle(Math.PI/2 + Math.atan(m));
                double rho = -Math.abs(-p1.y + m*p1.x) / Math.sqrt(Math.pow(m, 2) + 1);
                return new LineParameters(rho, theta);
            } else {
                double theta = normalizeAngle(Math.PI/2 + Math.atan(m));
                double rho = Math.abs(-p1.y + m*p1.x) / Math.sqrt(Math.pow(m, 2) + 1);
                return new LineParameters(rho, theta);
            }
        }
        double theta = Math.PI/180 * 90;
        double rho = Math.abs(-p1.y + m*p1.x) / Math.sqrt(Math.pow(m, 2) + 1);
        return new LineParameters(rho, theta);
    }

    private static double normalizeAngle(double angle) {
        double newAngle = angle;
        while (newAngle <= -Math.PI) newAngle += Math.PI * 2;
        while (newAngle > Math.PI) newAngle -= Math.PI * 2;
        return newAngle;
    }

    static List<LineParameters> filterSimilarLines(List<LineParameters> lines, double rhoTolerance, double thetaTolerance) {
        List<List<LineParameters>> chunks = new ArrayList<>();
        for (LineParameters line : lines) {
            boolean addedToAChunk = false;
            for (int chunkIndex = 0; chunkIndex < chunks.size(); chunkIndex++) {
                LineParameters firstLineFromChunk = chunks.get(chunkIndex).get(0);
                if (areLinesSimilar(line, firstLineFromChunk, rhoTolerance, thetaTolerance)) {
                    chunks.get(chunkIndex).add(line);
                    addedToAChunk = true;
                    break;
                }
            }
            if (!addedToAChunk) {
                chunks.add(new ArrayList<>(Collections.singletonList(line)));
            }
        }
        List<LineParameters> result = new ArrayList<>();
        for (List<LineParameters> linesParameters : chunks) {
            // No average calculation - just one parameter of chunk
            result.add(linesParameters.get(0));
//            double sumRho = 0.0, sumTheta = 0.0;
//            for (LineParameters singleLineParameter : linesParameters) {
//                sumRho += singleLineParameter.getRho();
//                sumTheta += singleLineParameter.getTheta();
//            }
//            result.add(new LineParameters(sumRho / linesParameters.size(), sumTheta / linesParameters.size()));
        }
        return result;
    }

    static boolean areLinesSimilar(LineParameters l1, LineParameters l2, double rhoTolerance, double thetaTolerance) {
        if (linesHasCloseAbsoluteParameters(l1, l2, rhoTolerance, thetaTolerance)) return true;
        // Maybe some theta is different. Let's try moving some coordinates
        LineParameters l1t = transformCoordinates(l1);
        LineParameters l2t = transformCoordinates(l2);
        return linesHasCloseAbsoluteParameters(l1t, l2t, rhoTolerance, thetaTolerance);
    }

    private static boolean linesHasCloseAbsoluteParameters(LineParameters l1, LineParameters l2, double rhoTolerance, double thetaTolerance) {
        // 1 - Rhos are too much different - not similar
        if (Math.abs(l1.getRho() - l2.getRho()) > rhoTolerance) return false;

        // 2 - Thetas are close, they are similar
        return Math.abs(l1.getTheta() - l2.getTheta()) <= thetaTolerance;
    }

    private static LineParameters transformCoordinates(LineParameters line) {
        if (line.getTheta() > Math.PI / 2) {
            double newTheta = line.getTheta() - Math.PI;
            double newRho = -line.getRho();
            return new LineParameters(newRho, newTheta);
        } else {
            return line;
        }
    }

    static boolean areThetasClose(double firstTheta, double secondTheta, double thetaTolerance) {
        // 1 - Thetas are close, they are similar
        if (Math.abs(firstTheta - secondTheta) <= thetaTolerance) return true;

        // 2 - Maybe thetas are different like 179 deg and 0 deg
        if (Math.abs(Math.abs(firstTheta - secondTheta) - Math.PI) <= thetaTolerance) return true;

        // 3 - They are not similar
        return false;
    }

    // Organize by parallel lines - top, left, bottom, right
    static List<List<LineParameters>> groupByNearParallelLines(List<LineParameters> lines, double thetaTolerance) {
        List<List<LineParameters>> dividedLines = new ArrayList<>();
        for (LineParameters line : lines) {
            boolean addedToAChunk = false;
            for (int chunkIndex = 0; chunkIndex < dividedLines.size(); chunkIndex++) {
                LineParameters firstDividedLineFromChunk = dividedLines.get(chunkIndex).get(0);
                if (areThetasClose(line.getTheta(), firstDividedLineFromChunk.getTheta(), thetaTolerance)) {
                    dividedLines.get(chunkIndex).add(line);
                    addedToAChunk = true;
                    break;
                }
            }
            if (!addedToAChunk) {
                dividedLines.add(new ArrayList<>(Collections.singletonList(line)));
            }
        }
        return dividedLines;
    }

    static Point[] getIntersectionsFromParallelLines(List<List<LineParameters>> lineGroups) {
        Point[] result = new Point[4];
        result[0] = getIntersection(lineGroups.get(0).get(0), lineGroups.get(1).get(1));
        result[1] = getIntersection(lineGroups.get(0).get(0), lineGroups.get(1).get(0));
        result[2] = getIntersection(lineGroups.get(0).get(1), lineGroups.get(1).get(0));
        result[3] = getIntersection(lineGroups.get(0).get(1), lineGroups.get(1).get(1));
        return result;
    }

    private static Point getIntersection(LineParameters l1, LineParameters l2) {
        Point[] pointsFromL1 = getTwoPointsOfLine(l1);
        Point[] pointsFromL2 = getTwoPointsOfLine(l2);
        Point p1 = pointsFromL1[0];
        Point p2 = pointsFromL1[1];
        Point p3 = pointsFromL2[0];
        Point p4 = pointsFromL2[1];
        // https://en.wikipedia.org/wiki/Line–line_intersection
        double t = ((p1.x-p3.x)*(p3.y-p4.y) - (p1.y-p3.y)*(p3.x-p4.x))/((p1.x-p2.x)*(p3.y-p4.y) - (p1.y-p2.y)*(p3.x-p4.x));
        double x = p1.x + t*(p2.x - p1.x);
        double y = p1.y + t*(p2.y - p1.y);
        return new Point(x, y);
    }

    private static Point[] getTwoPointsOfLine(LineParameters line) {
        double x1 = Math.cos(line.getTheta())*line.getRho();
        double y1 = Math.sin(line.getTheta())*line.getRho();
        double theta2 = Math.PI/2 - line.getTheta();
        double deltaX = 100.0*Math.cos(theta2);
        double deltaY = -100.0*Math.sin(theta2);
        double x2 = x1 + deltaX;
        double y2 = y1 + deltaY;
        Point[] result = new Point[2];
        result[0] = new Point(x1, y1);
        result[1] = new Point(x2, y2);
        return result;
    }

    static String printablePoint(Point point) {
        return "(" + (int)point.x + ", " + (int)point.y + ")";
    }
}
