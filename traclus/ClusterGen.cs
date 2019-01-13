using System;
using System.Collections.Generic;

namespace Traclus {
    public class ClusterGen {

        public TraClusterDoc m_document;

        private double m_epsParam;
        private int m_minLnsParam;
        private int m_nTotalLineSegments;
        private int m_currComponentId;
        // the number of dense components discovered until now
        private List<int> m_componentIdArray = new List<int>();
        // the list of line segment clusters
        private LineSegmentCluster[] m_lineSegmentClusters;
        // programming trick: avoid frequent execution of the new and delete operations
        private CMDPoint m_startPoint1, m_endPoint1, m_startPoint2, m_endPoint2;
        private CMDPoint m_vector1; //  = new CMDPoint(m_document.m_nDimensions);
        private CMDPoint m_vector2; // = new CMDPoint(m_document.m_nDimensions);;
        private CMDPoint m_projectionPoint; // = new CMDPoint( m_document.m_nDimensions);;
        double m_coefficient;

        private List<LineSegmentId> m_idArray = new List<ClusterGen.LineSegmentId>();
        private List<CMDPoint> m_lineSegmentPointArray = new List<CMDPoint>();

        // used for performing the DBSCAN algorithm
        public const int UNCLASSIFIED = -2;
        public const int NOISE = -1;

        private const double MIN_LINESEGMENT_LENGTH = 50.0;

        private const int MDL_COST_ADWANTAGE = 25;
        private const int INT_MAX = int.MaxValue;
        // used for InsertClusterPoint() and ReplaceClusterPoint() 
        enum PointLocation {
            HEAD, TAIL
        }

        class LineSegmentId {

            public int trajectoryId;
            public int order;
        }

        class CandidateClusterPoint {

            public double orderingValue;
            public int lineSegmentId;
            public bool startPointFlag;

        }

        class LineSegmentCluster {

            public int lineSegmentClusterId;
            public int nLineSegments;
            public CMDPoint avgDirectionVector;
            public double cosTheta, sinTheta;
            public List<CandidateClusterPoint> candidatePointList = new List<ClusterGen.CandidateClusterPoint>();
            public int nClusterPoints;
            public List<CMDPoint> clusterPointArray = new List<CMDPoint>();
            public int nTrajectories;
            public List<int> trajectoryIdList = new List<int>();
            public bool enabled;
        }
        // this default constructor should be never used	
        public ClusterGen() {

        }
        // use the following constructor instead
        public ClusterGen(TraClusterDoc document) {
            m_document = document;

            m_startPoint1 = new CMDPoint(m_document.m_nDimensions);
            m_startPoint2 = new CMDPoint(m_document.m_nDimensions);
            m_endPoint1 = new CMDPoint(m_document.m_nDimensions);
            m_endPoint2 = new CMDPoint(m_document.m_nDimensions);

            m_vector1 = new CMDPoint(m_document.m_nDimensions);
            m_vector2 = new CMDPoint(m_document.m_nDimensions);
            m_projectionPoint = new CMDPoint(m_document.m_nDimensions);


            m_idArray.Clear();
            m_lineSegmentPointArray.Clear();

        }

        public bool constructCluster() {
            // this step consists of two sub-steps
            // notice that the result of the previous sub-step is used in the following sub-steps
            if (!constructLineSegmentCluster()) {
                return false;
            }
            if (!storeLineSegmentCluster()) {
                return false;
            }
            return true;
        }

        public bool partitionTrajectory() {

            for (int i = 0; i < m_document.m_trajectoryList.Count; i++) {
                Trajectory pTrajectory = m_document.m_trajectoryList[i];

                findOptimalPartition(pTrajectory);

                m_document.m_trajectoryList[i] = pTrajectory;
            }
            if (!storeClusterComponentIntoIndex()) {
                return false;
            }
            return true;
        }

        public bool performDBSCAN(double eps, int minLns) {

            m_epsParam = eps;
            m_minLnsParam = minLns;

            m_currComponentId = 0;

            for (int i = 0; i < m_nTotalLineSegments; i++) {
                m_componentIdArray.Add(UNCLASSIFIED);
            }

            for (int i = 0; i < m_nTotalLineSegments; i++) {
                if (m_componentIdArray[i] == UNCLASSIFIED && expandDenseComponent(i, m_currComponentId, eps, minLns)) {
                    m_currComponentId++;
                }
            }
            return true;
        }


        private bool storeClusterComponentIntoIndex() {

            int nDimensions = m_document.m_nDimensions;
            CMDPoint startPoint;
            CMDPoint endPoint;

            m_nTotalLineSegments = 0;
            for (int i = 0; i < m_document.m_trajectoryList.Count; i++) {
                Trajectory pTrajectory = m_document.m_trajectoryList[i];

                for (int j = 0; j < pTrajectory.getM_nPartitionPoints() - 1; j++) {
                    // convert an n-dimensional line segment into a 2n-dimensional point
                    // i.e., the first n-dimension: the start point
                    //       the last n-dimension: the end point
                    startPoint = pTrajectory.getM_partitionPointArray()[j];
                    endPoint = pTrajectory.getM_partitionPointArray()[j + 1];

                    if (measureDistanceFromPointToPoint(startPoint, endPoint) < MIN_LINESEGMENT_LENGTH) {
                        continue;
                    }
                    m_nTotalLineSegments++;

                    CMDPoint lineSegmentPoint = new CMDPoint(nDimensions * 2);
                    for (int m = 0; m < nDimensions; m++) {
                        lineSegmentPoint.setM_coordinate(m, startPoint.getM_coordinate(m));
                        lineSegmentPoint.setM_coordinate(nDimensions + m, endPoint.getM_coordinate(m));
                    }

                    LineSegmentId id = new LineSegmentId();
                    id.trajectoryId = pTrajectory.getM_trajectoryId();
                    id.order = j;

                    m_idArray.Add(id);
                    m_lineSegmentPointArray.Add(lineSegmentPoint);
                }
            }

            return true;
        }

        private void findOptimalPartition(Trajectory pTrajectory) {

            int nPoints = pTrajectory.getM_nPoints();
            int startIndex = 0, length;
            int fullPartitionMDLCost, partialPartitionMDLCost;

            // add the start point of a trajectory
            CMDPoint startP = pTrajectory.getM_pointArray()[0];
            pTrajectory.addPartitionPointToArray(startP);

            for (; ; ) {
                fullPartitionMDLCost = partialPartitionMDLCost = 0;

                for (length = 1; startIndex + length < nPoints; length++) {
                    // compute the total length of a trajectory
                    fullPartitionMDLCost += computeModelCost(pTrajectory, startIndex + length - 1, startIndex + length);

                    // compute the sum of (1) the length of a cluster component and 
                    // 					 (2) the perpendicular and angle distances
                    partialPartitionMDLCost = computeModelCost(pTrajectory, startIndex, startIndex + length) +
                            computeEncodingCost(pTrajectory, startIndex, startIndex + length);

                    if (fullPartitionMDLCost + MDL_COST_ADWANTAGE < partialPartitionMDLCost) {

                        pTrajectory.addPartitionPointToArray(pTrajectory.getM_pointArray()[startIndex + length - 1]);
                        startIndex = startIndex + length - 1;
                        length = 0;
                        break;
                    }
                }
                // if we reach at the end of a trajectory
                if (startIndex + length >= nPoints) {
                    break;
                }
            }

            // add the end point of a trajectory
            pTrajectory.addPartitionPointToArray(pTrajectory.getM_pointArray()[nPoints - 1]);

            return;
        }

        private double LOG2(double x) {
            return Math.Log(x) / Math.Log(2);
        }

        private int computeModelCost(Trajectory pTrajectory, int startPIndex, int endPIndex) {

            CMDPoint lineSegmentStart = pTrajectory.getM_pointArray()[startPIndex];
            CMDPoint lineSegmentEnd = pTrajectory.getM_pointArray()[endPIndex];

            double distance = measureDistanceFromPointToPoint(lineSegmentStart, lineSegmentEnd);

            if (distance < 1.0) {
                distance = 1.0;         // to take logarithm
            }

            return (int)Math.Ceiling(LOG2(distance));

        }

        private int computeEncodingCost(Trajectory pTrajectory, int startPIndex, int endPIndex) {

            CMDPoint clusterComponentStart;
            CMDPoint clusterComponentEnd;
            CMDPoint lineSegmentStart;
            CMDPoint lineSegmentEnd;
            double perpendicularDistance;
            double angleDistance;
            int encodingCost = 0;

            clusterComponentStart = pTrajectory.getM_pointArray()[startPIndex];
            clusterComponentEnd = pTrajectory.getM_pointArray()[endPIndex];

            for (int i = startPIndex; i < endPIndex; i++) {
                lineSegmentStart = pTrajectory.getM_pointArray()[i];
                lineSegmentEnd = pTrajectory.getM_pointArray()[i + 1];

                perpendicularDistance = measurePerpendicularDistance(clusterComponentStart,
                        clusterComponentEnd, lineSegmentStart, lineSegmentEnd);
                angleDistance = measureAngleDisntance(clusterComponentStart,
                        clusterComponentEnd, lineSegmentStart, lineSegmentEnd);

                if (perpendicularDistance < 1.0) perpendicularDistance = 1.0;   //  to take logarithm
                if (angleDistance < 1.0) angleDistance = 1.0;                   //  to take logarithm

                encodingCost += (int)(Math.Ceiling(LOG2(perpendicularDistance)) + Math.Ceiling(LOG2(angleDistance)));
            }
            return encodingCost;

        }
        private double measurePerpendicularDistance(CMDPoint s1, CMDPoint e1, CMDPoint s2, CMDPoint e2) {

            //  we assume that the first line segment is longer than the second one
            double distance1;   //  the distance from a start point to the cluster component
            double distance2;   //  the distance from an end point to the cluster component

            distance1 = measureDistanceFromPointToLineSegment(s1, e1, s2);
            distance2 = measureDistanceFromPointToLineSegment(s1, e1, e2);

            //  if the first line segment is exactly the same as the second one, 
            //  the perpendicular distance should be zero
            if (distance1 == 0.0 && distance2 == 0.0) return 0.0;

            //  return (d1^2 + d2^2) / (d1 + d2) as the perpendicular distance
            return ((Math.Pow(distance1, 2) + Math.Pow(distance2, 2)) / (distance1 + distance2));

        }

        private double measureDistanceFromPointToLineSegment(CMDPoint s, CMDPoint e, CMDPoint p) {

            int nDimensions = p.getM_nDimensions();

            //  NOTE: the variables m_vector1 and m_vector2 are declared as member variables

            //  construct two vectors as follows
            //  1. the vector connecting the start point of the cluster component and a given point
            //  2. the vector representing the cluster component
            for (int i = 0; i < nDimensions; i++)
            {
                m_vector1.setM_coordinate(i, p.getM_coordinate(i) - s.getM_coordinate(i));
                m_vector2.setM_coordinate(i, e.getM_coordinate(i) - s.getM_coordinate(i));
            }

            //  a coefficient (0 <= b <= 1)
            m_coefficient = computeInnerProduct(m_vector1, m_vector2) / computeInnerProduct(m_vector2, m_vector2);

            //  the projection on the cluster component from a given point
            //  NOTE: the variable m_projectionPoint is declared as a member variable

            for (int i = 0; i < nDimensions; i++) {
                m_projectionPoint.setM_coordinate(i, s.getM_coordinate(i) + m_coefficient * m_vector2.getM_coordinate(i));
            }

            //  return the distance between the projection point and the given point
            return measureDistanceFromPointToPoint(p, m_projectionPoint);

        }

        private double measureDistanceFromPointToPoint(CMDPoint point1, CMDPoint point2) {

            int nDimensions = point1.getM_nDimensions();
            double squareSum = 0.0;

            for (int i = 0; i < nDimensions; i++) {
                squareSum += Math.Pow((point2.getM_coordinate(i) - point1.getM_coordinate(i)), 2);
            }
            return Math.Sqrt(squareSum);

        }

        private double computeVectorLength(CMDPoint vector) {

            int nDimensions = vector.getM_nDimensions();
            double squareSum = 0.0;

            for (int i = 0; i < nDimensions; i++) {
                squareSum += Math.Pow(vector.getM_coordinate(i), 2);
            }

            return Math.Sqrt(squareSum);
        }

        private double computeInnerProduct(CMDPoint vector1, CMDPoint vector2) {
            int nDimensions = vector1.getM_nDimensions();
            double innerProduct = 0.0;

            for (int i = 0; i < nDimensions; i++) {
                innerProduct += (vector1.getM_coordinate(i) * vector2.getM_coordinate(i));
            }

            return innerProduct;
        }
        private double measureAngleDisntance(CMDPoint s1, CMDPoint e1, CMDPoint s2, CMDPoint e2) {

            int nDimensions = s1.getM_nDimensions();

            //  NOTE: the variables m_vector1 and m_vector2 are declared as member variables
            //  construct two vectors representing the cluster component and a line segment, respectively
            for (int i = 0; i < nDimensions; i++) {
                m_vector1.setM_coordinate(i, e1.getM_coordinate(i) - s1.getM_coordinate(i));
                m_vector2.setM_coordinate(i, e2.getM_coordinate(i) - s2.getM_coordinate(i));
            }

            //  we assume that the first line segment is longer than the second one
            //  i.e., vectorLength1 >= vectorLength2
            double vectorLength1 = computeVectorLength(m_vector1);
            double vectorLength2 = computeVectorLength(m_vector2);

            //  if one of two vectors is a point, the angle distance becomes zero
            if (vectorLength1 == 0.0 || vectorLength2 == 0.0) return 0.0;

            //  compute the inner product of the two vectors
            double innerProduct = computeInnerProduct(m_vector1, m_vector2);

            //  compute the angle between two vectors by using the inner product
            double cosTheta = innerProduct / (vectorLength1 * vectorLength2);
            //  compensate the computation error (e.g., 1.00001)
            //  cos(theta) should be in the range [-1.0, 1.0]
            //  START ...
            if (cosTheta > 1.0) cosTheta = 1.0;
            if (cosTheta < -1.0) cosTheta = -1.0;
            //  ... END
            double sinTheta = Math.Sqrt(1 - Math.Pow(cosTheta, 2));
            //  if 90 <= theta <= 270, the angle distance becomes the length of the line segment
            //  if (cosTheta < -1.0) sinTheta = 1.0;

            return (vectorLength2 * sinTheta);
        }

        private bool expandDenseComponent(int index, int componentId, double eps, int minDensity) {

            HashSet<int> seeds = new HashSet<int>();
            HashSet<int> seedResult = new HashSet<int>();

            extractStartAndEndPoints(index, m_startPoint1, m_endPoint1);
            computeEPSNeighborhood(m_startPoint1, m_endPoint1, eps, seeds);

            if (seeds.Count < minDensity) { //  not a core line segment
                m_componentIdArray[index] = NOISE;
                return false;
            }
            // else...
            foreach (int i in seeds) {
                m_componentIdArray[i] = componentId;
            }
            seeds.Remove(index);
            while (seeds.Count > 0) {
                var iter = seeds.GetEnumerator();
                iter.MoveNext();
                int currIndex = iter.Current;
                extractStartAndEndPoints(currIndex, m_startPoint1, m_endPoint1);
                computeEPSNeighborhood(m_startPoint1, m_endPoint1, eps, seedResult);

                if (seedResult.Count >= minDensity) {
                    foreach (int i in seedResult) {
                        if (m_componentIdArray[i] == UNCLASSIFIED ||
                                m_componentIdArray[i] == NOISE) {
                            if (m_componentIdArray[i] == UNCLASSIFIED) {
                                seeds.Add(i);
                            }
                            m_componentIdArray[i] = componentId;
                        }
                    }
                }

                seeds.Remove(currIndex);
            }

            return true;
        }

        bool constructLineSegmentCluster() {
            int nDimensions = m_document.m_nDimensions;
            m_lineSegmentClusters = new LineSegmentCluster[m_currComponentId];

            //  initialize the list of line segment clusters
            //  START ...
            for (int i = 0; i < m_currComponentId; i++) {
                m_lineSegmentClusters[i] = new LineSegmentCluster();
                m_lineSegmentClusters[i].avgDirectionVector = new CMDPoint(nDimensions);
                m_lineSegmentClusters[i].lineSegmentClusterId = i;
                m_lineSegmentClusters[i].nLineSegments = 0;
                m_lineSegmentClusters[i].nClusterPoints = 0;
                m_lineSegmentClusters[i].nTrajectories = 0;
                m_lineSegmentClusters[i].enabled = false;
            }
            //  ... END

            //  accumulate the direction vector of a line segment
            for (int i = 0; i < m_nTotalLineSegments; i++) {
                int componentId = m_componentIdArray[i];
                if (componentId >= 0) {
                    for (int j = 0; j < nDimensions; j++) {
                        double difference = m_lineSegmentPointArray[i].getM_coordinate(nDimensions + j)
                                - m_lineSegmentPointArray[i].getM_coordinate(j);
                        double currSum = m_lineSegmentClusters[componentId].avgDirectionVector.getM_coordinate(j)
                                + difference;
                        m_lineSegmentClusters[componentId].avgDirectionVector.setM_coordinate(j, currSum);
                    }
                    m_lineSegmentClusters[componentId].nLineSegments++;
                }
            }

            //  compute the average direction vector of a line segment cluster
            //  START ...
            double vectorLength1, vectorLength2, innerProduct;
            double cosTheta, sinTheta;

            m_vector2.setM_coordinate(0, 1.0);
            m_vector2.setM_coordinate(1, 0.0);

            for (int i = 0; i < m_currComponentId; i++) {
                LineSegmentCluster clusterEntry = m_lineSegmentClusters[i];

                for (int j = 0; j < nDimensions; j++) {
                    clusterEntry.avgDirectionVector.setM_coordinate(j, clusterEntry.avgDirectionVector.getM_coordinate(j) / (double)clusterEntry.nLineSegments);
                }
                vectorLength1 = computeVectorLength(clusterEntry.avgDirectionVector);
                vectorLength2 = 1.0;

                innerProduct = computeInnerProduct(clusterEntry.avgDirectionVector, m_vector2);
                cosTheta = innerProduct / (vectorLength1 * vectorLength2);
                if (cosTheta > 1.0) cosTheta = 1.0;
                if (cosTheta < -1.0) cosTheta = -1.0;
                sinTheta = Math.Sqrt(1 - Math.Pow(cosTheta, 2));

                if (clusterEntry.avgDirectionVector.getM_coordinate(1) < 0) {
                    sinTheta = -sinTheta;
                }

                clusterEntry.cosTheta = cosTheta;
                clusterEntry.sinTheta = sinTheta;

            }
            //  ... END

            //  summarize the information about line segment clusters
            //  the structure for summarization is as follows
            //  [lineSegmentClusterId, nClusterPoints, clusterPointArray, nTrajectories, { trajectoryId, ... }]
            for (int i = 0; i < m_nTotalLineSegments; i++) {
                if (m_componentIdArray[i] >= 0)     //  if the componentId < 0, it is a noise
                    RegisterAndUpdateLineSegmentCluster(m_componentIdArray[i], i);
            }

            HashSet<int> trajectories = new HashSet<int>();
            for (int i = 0; i < m_currComponentId; i++) {
                LineSegmentCluster clusterEntry = (m_lineSegmentClusters[i]);

                //  a line segment cluster must have trajectories more than the minimum threshold
                if (clusterEntry.nTrajectories >= m_minLnsParam) {
                    clusterEntry.enabled = true;
                    // m_lineSegmentClusters[i].enabled = true;
                    //  DEBUG: count the number of trajectories that belong to clusters
                    for (int j = 0; j < clusterEntry.trajectoryIdList.Count; j++) {
                        trajectories.Add(clusterEntry.trajectoryIdList[j]);
                    }

                    computeRepresentativeLines(clusterEntry);
                    // computeRepresentativeLines(m_lineSegmentClusters[i]);
                } else {
                    clusterEntry.candidatePointList.Clear();
                    clusterEntry.clusterPointArray.Clear();
                    clusterEntry.trajectoryIdList.Clear();
                }

            }
            //  DEBUG: compute the ratio of trajectories that belong to clusters
            m_document.m_clusterRatio = (double)trajectories.Count / (double)m_document.m_nTrajectories;

            return true;
        }


        private void computeRepresentativeLines(LineSegmentCluster clusterEntry) {

            HashSet<int> lineSegments = new HashSet<int>();
            HashSet<int> insertionList = new HashSet<int>();
            HashSet<int> deletionList = new HashSet<int>();

            int iter = 0;
            CandidateClusterPoint candidatePoint, nextCandidatePoint;
            double prevOrderingValue = 0.0;

            int nClusterPoints = 0;
            lineSegments.Clear();

            //  sweep the line segments in a line segment cluster

            while (iter != (clusterEntry.candidatePointList.Count - 1) && clusterEntry.candidatePointList.Count > 0) {
                insertionList.Clear();
                deletionList.Clear();

                do {
                    candidatePoint = clusterEntry.candidatePointList[iter];
                    iter++;
                    //  check whether this line segment has begun or not
                    if (!lineSegments.Contains(candidatePoint.lineSegmentId)) {
                        // iter1 = lineSegments.find(candidatePoint.lineSegmentId);
                        // if (iter1 == lineSegments.end())	{				//  if there is no matched element,
                        insertionList.Add(candidatePoint.lineSegmentId);        //  this line segment begins at this point
                        lineSegments.Add(candidatePoint.lineSegmentId);
                    } else {                        //  if there is a matched element,
                        deletionList.Add(candidatePoint.lineSegmentId);     //  this line segment ends at this point
                    }
                    //  check whether the next line segment begins or ends at the same point
                    if (iter != (clusterEntry.candidatePointList.Count - 1)) {
                        nextCandidatePoint = clusterEntry.candidatePointList[iter];
                    } else {
                        break;
                    }
                } while (candidatePoint.orderingValue == nextCandidatePoint.orderingValue);

                //  check if a line segment is connected to another line segment in the same trajectory
                //  if so, delete one of the line segments to remove duplicates
                foreach (int a in insertionList) {
                    foreach (int b in deletionList) {
                        if (m_idArray[a].trajectoryId
                                == m_idArray[b].trajectoryId) {
                            lineSegments.Remove(b);
                            deletionList.Remove(b);
                            break;
                        }
                    }
                }

                // if the current density exceeds a given threshold
                if (lineSegments.Count >= m_minLnsParam) {
                    if (Math.Abs(candidatePoint.orderingValue - prevOrderingValue) > ((double)MIN_LINESEGMENT_LENGTH / 1.414)) {
                        computeAndRegisterClusterPoint(clusterEntry, candidatePoint.orderingValue, lineSegments);
                        prevOrderingValue = candidatePoint.orderingValue;
                        nClusterPoints++;
                    }
                }

                //  delete the line segment that is not connected to another line segment
                foreach (int b in deletionList) {
                    lineSegments.Remove(b);
                }
            }

            if (nClusterPoints >= 2) {
                clusterEntry.nClusterPoints = nClusterPoints;
            } else {
                //  there is no representative trend in this line segment cluster
                clusterEntry.enabled = false;
                clusterEntry.candidatePointList.Clear();
                clusterEntry.clusterPointArray.Clear();
                clusterEntry.trajectoryIdList.Clear();
            }
            return;
        }

        private void computeAndRegisterClusterPoint(
                LineSegmentCluster clusterEntry,
                double currValue,
                HashSet<int> lineSegments) {
            int nDimensions = m_document.m_nDimensions;
            int nLineSegmentsInSet = lineSegments.Count;
            CMDPoint clusterPoint = new CMDPoint(nDimensions);
            CMDPoint sweepPoint = new CMDPoint(nDimensions);

            foreach (int iter in lineSegments) {
                // get the sweep point of each line segment
                // this point is parallel to the current value of the sweeping direction
                getSweepPointOfLineSegment(clusterEntry, currValue, iter, sweepPoint);
                for (int i = 0; i < nDimensions; i++) {
                    clusterPoint.setM_coordinate(i, clusterPoint.getM_coordinate(i) +
                            (sweepPoint.getM_coordinate(i) / (double)nLineSegmentsInSet));
                }
            }

            // NOTE: this program code works only for the 2-dimensional data
            double origX, origY;
            origX = GET_X_REV_ROTATION(clusterPoint.getM_coordinate(0), clusterPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
            origY = GET_Y_REV_ROTATION(clusterPoint.getM_coordinate(0), clusterPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
            clusterPoint.setM_coordinate(0, origX);
            clusterPoint.setM_coordinate(1, origY);

            // register the obtained cluster point (i.e., the average of all the sweep points)
            clusterEntry.clusterPointArray.Add(clusterPoint);

            return;
        }

        private void getSweepPointOfLineSegment(LineSegmentCluster clusterEntry,
                double currValue, int lineSegmentId, CMDPoint sweepPoint) {

            CMDPoint lineSegmentPoint = m_lineSegmentPointArray[lineSegmentId];     //  2n-dimensional point
            double coefficient;

            //  NOTE: this program code works only for the 2-dimensional data
            double newStartX, newEndX, newStartY, newEndY;
            newStartX = GET_X_ROTATION(lineSegmentPoint.getM_coordinate(0), lineSegmentPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
            newEndX = GET_X_ROTATION(lineSegmentPoint.getM_coordinate(2), lineSegmentPoint.getM_coordinate(3), clusterEntry.cosTheta, clusterEntry.sinTheta);
            newStartY = GET_Y_ROTATION(lineSegmentPoint.getM_coordinate(0), lineSegmentPoint.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
            newEndY = GET_Y_ROTATION(lineSegmentPoint.getM_coordinate(2), lineSegmentPoint.getM_coordinate(3), clusterEntry.cosTheta, clusterEntry.sinTheta);

            coefficient = (currValue - newStartX) / (newEndX - newStartX);
            sweepPoint.setM_coordinate(0, currValue);
            sweepPoint.setM_coordinate(1, newStartY + coefficient * (newEndY - newStartY));

            return;
        }


        private double GET_X_ROTATION(double _x, double _y, double _cos, double _sin) {
            return ((_x) * (_cos) + (_y) * (_sin));
        }
        private double GET_Y_ROTATION(double _x, double _y, double _cos, double _sin) {
            return (-(_x) * (_sin) + (_y) * (_cos));
        }
        private double GET_X_REV_ROTATION(double _x, double _y, double _cos, double _sin) {
            return ((_x) * (_cos) - (_y) * (_sin));
        }
        private double GET_Y_REV_ROTATION(double _x, double _y, double _cos, double _sin) {
            return ((_x) * (_sin) + (_y) * (_cos));
        }

        private void RegisterAndUpdateLineSegmentCluster(int componentId, int lineSegmentId) {
            LineSegmentCluster clusterEntry = m_lineSegmentClusters[componentId];

            //  the start and end values of the first dimension (e.g., the x value in the 2-dimension)
            //  NOTE: this program code works only for the 2-dimensional data

            CMDPoint aLineSegment = m_lineSegmentPointArray[lineSegmentId];
            double orderingValue1 = GET_X_ROTATION(aLineSegment.getM_coordinate(0),
                    aLineSegment.getM_coordinate(1), clusterEntry.cosTheta, clusterEntry.sinTheta);
            double orderingValue2 = GET_X_ROTATION(aLineSegment.getM_coordinate(2),
                    aLineSegment.getM_coordinate(3), clusterEntry.cosTheta, clusterEntry.sinTheta);

            CandidateClusterPoint existingCandidatePoint, newCandidatePoint1, newCandidatePoint2;
            int i, j;
            //  sort the line segment points by the coordinate of the first dimension
            //  simply use the insertion sort algorithm
            //  START ...
            int iter1 = 0;
            for (i = 0; i < clusterEntry.candidatePointList.Count; i++) {
                existingCandidatePoint = clusterEntry.candidatePointList[iter1];
                if (existingCandidatePoint.orderingValue >= orderingValue1) {
                    break;
                }
                iter1++;
            }
            newCandidatePoint1 = new CandidateClusterPoint();

            newCandidatePoint1.orderingValue = orderingValue1;
            newCandidatePoint1.lineSegmentId = lineSegmentId;
            newCandidatePoint1.startPointFlag = true;
            if (i == 0) {
                clusterEntry.candidatePointList.Insert(0, newCandidatePoint1);
            } else if (i >= clusterEntry.candidatePointList.Count) {
                clusterEntry.candidatePointList.Add(newCandidatePoint1);
            } else {
                clusterEntry.candidatePointList.Insert(iter1, newCandidatePoint1);
            }
            int iter2 = 0;
            for (j = 0; j < clusterEntry.candidatePointList.Count; j++) {
                existingCandidatePoint = clusterEntry.candidatePointList[iter2];
                if (existingCandidatePoint.orderingValue >= orderingValue2) {
                    break;
                }
                iter2++;
            }

            newCandidatePoint2 = new CandidateClusterPoint();
            newCandidatePoint2.orderingValue = orderingValue2;
            newCandidatePoint2.lineSegmentId = lineSegmentId;
            newCandidatePoint2.startPointFlag = false;

            if (j == 0) {
                clusterEntry.candidatePointList.Insert(0, newCandidatePoint2);
            } else if (j >= clusterEntry.candidatePointList.Count) {
                clusterEntry.candidatePointList.Add(newCandidatePoint2);
            } else {
                clusterEntry.candidatePointList.Insert(iter2, newCandidatePoint2);
            }
            //  ... END

            int trajectoryId = m_idArray[lineSegmentId].trajectoryId;

            //  store the identifier of the trajectories that belong to this line segment cluster
            if (!clusterEntry.trajectoryIdList.Contains(trajectoryId)) {
                clusterEntry.trajectoryIdList.Add(trajectoryId);
                clusterEntry.nTrajectories++;
            }
            return;
        }
        private void computeEPSNeighborhood(CMDPoint startPoint, CMDPoint endPoint, double eps, HashSet<int> result) {
            result.Clear();
            for (int j = 0; j < m_nTotalLineSegments; j++) {
                extractStartAndEndPoints(j, m_startPoint2, m_endPoint2);
                double distance = computeDistanceBetweenTwoLineSegments(startPoint, endPoint, m_startPoint2, m_endPoint2);
                //  if the distance is below the threshold, this line segment belongs to the eps-neighborhood
                if (distance <= eps) result.Add(j);
            }
            return;
        }
        private double computeDistanceBetweenTwoLineSegments(CMDPoint startPoint1,
                CMDPoint endPoint1, CMDPoint startPoint2, CMDPoint endPoint2) {
            double perpendicularDistance = 0;
            double parallelDistance = 0;
            double angleDistance = 0;

            return subComputeDistanceBetweenTwoLineSegments(startPoint1, endPoint1, startPoint2, endPoint2, perpendicularDistance, parallelDistance, angleDistance);
        }


        private bool storeLineSegmentCluster() {

            int currClusterId = 0;

            for (int i = 0; i < m_currComponentId; i++) {

                if (!m_lineSegmentClusters[i].enabled) {
                    continue;
                }

                //  store the clusters constly identified
                //  START ...
                Cluster pClusterItem = new Cluster(currClusterId, m_document.m_nDimensions);

                for (int j = 0; j < m_lineSegmentClusters[i].nClusterPoints; j++) {
                    pClusterItem.addPointToArray(m_lineSegmentClusters[i].clusterPointArray[j]);
                }

                pClusterItem.setDensity(m_lineSegmentClusters[i].nTrajectories);

                m_document.m_clusterList.Add(pClusterItem);

                currClusterId++;    //  increase the number of const clusters
                                    //  ... END
            }

            m_document.m_nClusters = currClusterId;
            return true;
        }


        private double subComputeDistanceBetweenTwoLineSegments(CMDPoint startPoint1,
                CMDPoint endPoint1, CMDPoint startPoint2, CMDPoint endPoint2,
                double perpendicularDistance, double parallelDistance,
                double angleDistance) {

            double perDistance1, perDistance2;
            double parDistance1, parDistance2;
            double length1, length2;

            //  the length of the first line segment
            length1 = measureDistanceFromPointToPoint(startPoint1, endPoint1);
            //  the length of the second line segment
            length2 = measureDistanceFromPointToPoint(startPoint2, endPoint2);

            //  compute the perpendicular distance and the parallel distance
            //  START ...
            if (length1 > length2) {
                perDistance1 = measureDistanceFromPointToLineSegment(startPoint1, endPoint1, startPoint2);
                if (m_coefficient < 0.5) parDistance1 = measureDistanceFromPointToPoint(startPoint1, m_projectionPoint);
                else parDistance1 = measureDistanceFromPointToPoint(endPoint1, m_projectionPoint);

                perDistance2 = measureDistanceFromPointToLineSegment(startPoint1, endPoint1, endPoint2);
                if (m_coefficient < 0.5) parDistance2 = measureDistanceFromPointToPoint(startPoint1, m_projectionPoint);
                else parDistance2 = measureDistanceFromPointToPoint(endPoint1, m_projectionPoint);
            } else {
                perDistance1 = measureDistanceFromPointToLineSegment(startPoint2, endPoint2, startPoint1);
                if (m_coefficient < 0.5) parDistance1 = measureDistanceFromPointToPoint(startPoint2, m_projectionPoint);
                else parDistance1 = measureDistanceFromPointToPoint(endPoint2, m_projectionPoint);

                perDistance2 = measureDistanceFromPointToLineSegment(startPoint2, endPoint2, endPoint1);
                if (m_coefficient < 0.5) parDistance2 = measureDistanceFromPointToPoint(startPoint2, m_projectionPoint);
                else parDistance2 = measureDistanceFromPointToPoint(endPoint2, m_projectionPoint);

            }

            //  compute the perpendicular distance; take (d1^2 + d2^2) / (d1 + d2)
            if (!(perDistance1 == 0.0 && perDistance2 == 0.0)) {
                perpendicularDistance = ((Math.Pow(perDistance1, 2) + Math.Pow(perDistance2, 2)) / (perDistance1 + perDistance2));
            } else {
                perpendicularDistance = 0.0;
            }
            //  compute the parallel distance; take the minimum
            parallelDistance = (parDistance1 < parDistance2) ? parDistance1 : parDistance2;
            //  ... END

            //  compute the angle distance
            //  START ...
            //  MeasureAngleDisntance() assumes that the first line segment is longer than the second one
            if (length1 > length2) {
                angleDistance = measureAngleDisntance(startPoint1, endPoint1, startPoint2, endPoint2);
            } else {
                angleDistance = measureAngleDisntance(startPoint2, endPoint2, startPoint1, endPoint1);
            }
            //  ... END

            return (perpendicularDistance + parallelDistance + angleDistance);


        }
        private void extractStartAndEndPoints(int index, CMDPoint startPoint, CMDPoint endPoint) {//  for speedup
                                                                                                  //  compose the start and end points of the line segment
            for (int i = 0; i < m_document.m_nDimensions; i++) {
                startPoint.setM_coordinate(i, m_lineSegmentPointArray[index].getM_coordinate(i));
                endPoint.setM_coordinate(i, m_lineSegmentPointArray[index].getM_coordinate(m_document.m_nDimensions + i)); ;
            }
        }

        public bool estimateParameterValue(Parameter p) {

            double entropy, minEntropy = (double)INT_MAX;
            double eps, minEps = (double)INT_MAX;
            int totalSize, minTotalSize = INT_MAX;
            HashSet<int> seeds = new HashSet<int>();

            int[] EpsNeighborhoodSize = new int[m_nTotalLineSegments];

            for (eps = 20.0; eps <= 40.0; eps += 1.0) {
                entropy = 0.0;
                totalSize = 0;
                seeds.Clear();
                for (int i = 0; i < m_nTotalLineSegments; i++) {
                    extractStartAndEndPoints(i, m_startPoint1, m_endPoint1);
                    computeEPSNeighborhood(m_startPoint1, m_endPoint1, eps, seeds);
                    EpsNeighborhoodSize[i] = seeds.Count;
                    totalSize += seeds.Count;
                    seeds.Clear();
                }
                for (int i = 0; i < m_nTotalLineSegments; i++) {
                    entropy += ((double)EpsNeighborhoodSize[i] / (double)totalSize) * LOG2((double)EpsNeighborhoodSize[i] / (double)totalSize);
                }
                entropy = -entropy;

                if (entropy < minEntropy)
                {
                    minEntropy = entropy;
                    minTotalSize = totalSize;
                    minEps = eps;
                }
            }
            // setup output arguments
            p.epsParam = minEps;
            p.minLnsParam = (int)Math.Ceiling((double)minTotalSize / (double)m_nTotalLineSegments);
            return true;
        }

    }

}
