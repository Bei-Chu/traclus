using System;
using System.Collections.Generic;

namespace Traclus {
    public class ClusterGen {

        public TraClusterDoc m_document;

        private Parameter m_parameter = new Parameter();
        private int m_nTotalLineSegments;
        private int m_currComponentId;
        // the number of dense components discovered until now
        private List<int> m_componentIdArray = new List<int>();
        // the list of line segment clusters
        private LineSegmentCluster[] m_lineSegmentClusters;
        private Point2D m_projectionPoint; // = new CMDPoint( m_document.m_nDimensions);;
        double m_coefficient;

        private List<LineSegmentId> m_idArray = new List<LineSegmentId>();
        private List<Segment> m_lineSegmentPointArray = new List<Segment>();

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

        struct LineSegmentId {

            public int trajectoryId;
            public int order;
        }

        struct CandidateClusterPoint {

            public double orderingValue;
            public int lineSegmentId;
            public bool startPointFlag;

        }

        class LineSegmentCluster {

            public int lineSegmentClusterId;
            public int nLineSegments;
            public Point2D avgDirectionVector;
            public double cosTheta, sinTheta;
            public List<CandidateClusterPoint> candidatePointList = new List<CandidateClusterPoint>();
            public int nClusterPoints;
            public List<Point2D> clusterPointArray = new List<Point2D>();
            public int nTrajectories;
            public List<int> trajectoryIdList = new List<int>();
            public bool enabled;
        }

        // use the following constructor instead
        public ClusterGen(TraClusterDoc document) {

            m_document = document;
            m_parameter = document.m_parameter;

            m_projectionPoint = new Point2D(0, 0);

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

        public bool performDBSCAN() {

            m_currComponentId = 0;

            for (int i = 0; i < m_nTotalLineSegments; i++) {
                m_componentIdArray.Add(UNCLASSIFIED);
            }

            for (int i = 0; i < m_nTotalLineSegments; i++) {
                if (m_componentIdArray[i] == UNCLASSIFIED && expandDenseComponent(i, m_currComponentId, m_parameter.epsParam, m_parameter.minLnsParam)) {
                    m_currComponentId++;
                }
            }
            return true;
        }


        private bool storeClusterComponentIntoIndex() {

            Point2D startPoint;
            Point2D endPoint;

            m_nTotalLineSegments = 0;
            foreach (Trajectory pTrajectory in m_document.m_trajectoryList) {

                int nStartPoints = pTrajectory.getM_partitionPointArray().Count - 1;
                for (int j = 0; j < nStartPoints; j++) {
                    // convert an n-dimensional line segment into a 2n-dimensional point
                    // i.e., the first n-dimension: the start point
                    //       the last n-dimension: the end point
                    startPoint = pTrajectory.getM_partitionPointArray()[j];
                    endPoint = pTrajectory.getM_partitionPointArray()[j + 1];

                    if ((startPoint - endPoint).Length() < m_parameter.minSegmentLength) {
                        continue;
                    }
                    m_nTotalLineSegments++;

                    Segment lineSegmentPoint = new Segment(startPoint, endPoint);

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

            int nPoints = pTrajectory.getM_pointArray().Count;
            int startIndex = 0, length;
            int fullPartitionMDLCost, partialPartitionMDLCost;

            // add the start point of a trajectory
            Point2D startP = pTrajectory.getM_pointArray()[0];
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

                    if (fullPartitionMDLCost + m_parameter.MDLCostAdwantage < partialPartitionMDLCost) {

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

            Point2D lineSegmentStart = pTrajectory.getM_pointArray()[startPIndex];
            Point2D lineSegmentEnd = pTrajectory.getM_pointArray()[endPIndex];

            double distance = (lineSegmentStart - lineSegmentEnd).Length();

            if (distance < 1.0) {
                distance = 1.0;         // to take logarithm
            }

            return (int)Math.Ceiling(LOG2(distance));

        }

        private int computeEncodingCost(Trajectory pTrajectory, int startPIndex, int endPIndex) {

            Point2D clusterComponentStart;
            Point2D clusterComponentEnd;
            Point2D lineSegmentStart;
            Point2D lineSegmentEnd;
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
        private double measurePerpendicularDistance(Point2D s1, Point2D e1, Point2D s2, Point2D e2) {

            //  we assume that the first line segment is longer than the second one
            double distance1;   //  the distance from a start point to the cluster component
            double distance2;   //  the distance from an end point to the cluster component

            distance1 = measureDistanceFromPointToLineSegment(s1, e1, s2);
            distance2 = measureDistanceFromPointToLineSegment(s1, e1, e2);

            //  if the first line segment is exactly the same as the second one,
            //  the perpendicular distance should be zero
            if (distance1 == 0.0 && distance2 == 0.0) return 0.0;

            //  return (d1^2 + d2^2) / (d1 + d2) as the perpendicular distance
            return (distance1 * distance1 + distance2 * distance2) / (distance1 + distance2);

        }

        private double measureDistanceFromPointToLineSegment(Point2D s, Point2D e, Point2D p) {

            //  NOTE: the variables m_vector1 and m_vector2 are declared as member variables

            //  construct two vectors as follows
            //  1. the vector connecting the start point of the cluster component and a given point
            //  2. the vector representing the cluster component

            var m_vector1 = p - s;
            var m_vector2 = e - s;

            //  a coefficient (0 <= b <= 1)
            m_coefficient = m_vector1.Dot(m_vector2) / m_vector2.Dot(m_vector2);

            //  the projection on the cluster component from a given point
            //  NOTE: the variable m_projectionPoint is declared as a member variable

            m_projectionPoint = s + m_coefficient * m_vector2;

            //  return the distance between the projection point and the given point
            return (p - m_projectionPoint).Length();

        }

        private double measureAngleDisntance(Point2D s1, Point2D e1, Point2D s2, Point2D e2) {

            //  NOTE: the variables m_vector1 and m_vector2 are declared as member variables
            //  construct two vectors representing the cluster component and a line segment, respectively
            var m_vector1 = e1 - s1;
            var m_vector2 = e2 - s2;

            //  we assume that the first line segment is longer than the second one
            //  i.e., vectorLength1 >= vectorLength2
            double vectorLength1 = m_vector1.Length();
            double vectorLength2 = m_vector2.Length();

            //  if one of two vectors is a point, the angle distance becomes zero
            if (vectorLength1 == 0.0 || vectorLength2 == 0.0) return 0.0;

            //  compute the inner product of the two vectors
            double innerProduct = m_vector1.Dot(m_vector2);

            //  compute the angle between two vectors by using the inner product
            double cosTheta = innerProduct / (vectorLength1 * vectorLength2);
            //  compensate the computation error (e.g., 1.00001)
            //  cos(theta) should be in the range [-1.0, 1.0]
            //  START ...
            if (cosTheta > 1.0) cosTheta = 1.0;
            if (cosTheta < -1.0) cosTheta = -1.0;
            //  ... END
            double sinTheta = Math.Sqrt(1 - cosTheta * cosTheta);
            //  if 90 <= theta <= 270, the angle distance becomes the length of the line segment
            //  if (cosTheta < -1.0) sinTheta = 1.0;

            return (vectorLength2 * sinTheta);
        }

        private bool expandDenseComponent(int index, int componentId, double eps, int minDensity) {

            HashSet<int> seeds = new HashSet<int>();
            HashSet<int> seedResult = new HashSet<int>();

            computeEPSNeighborhood(m_lineSegmentPointArray[index].start, m_lineSegmentPointArray[index].end, eps, seeds);

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
                computeEPSNeighborhood(m_lineSegmentPointArray[currIndex].start, m_lineSegmentPointArray[currIndex].end, eps, seedResult);

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
            m_lineSegmentClusters = new LineSegmentCluster[m_currComponentId];

            //  initialize the list of line segment clusters
            //  START ...
            for (int i = 0; i < m_currComponentId; i++) {
                m_lineSegmentClusters[i] = new LineSegmentCluster();
                m_lineSegmentClusters[i].avgDirectionVector = new Point2D(0, 0);
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
                    m_lineSegmentClusters[componentId].avgDirectionVector += (m_lineSegmentPointArray[i].end - m_lineSegmentPointArray[i].start);
                    m_lineSegmentClusters[componentId].nLineSegments++;
                }
            }

            //  compute the average direction vector of a line segment cluster
            //  START ...
            double vectorLength1, vectorLength2, innerProduct;
            double cosTheta, sinTheta;

            var m_vector2 = new Point2D(1.0, 0.0);

            for (int i = 0; i < m_currComponentId; i++) {
                LineSegmentCluster clusterEntry = m_lineSegmentClusters[i];

                clusterEntry.avgDirectionVector /= (double)clusterEntry.nLineSegments;
                vectorLength1 = clusterEntry.avgDirectionVector.Length();
                vectorLength2 = 1.0;

                innerProduct = clusterEntry.avgDirectionVector.Dot(m_vector2);
                cosTheta = innerProduct / (vectorLength1 * vectorLength2);
                if (cosTheta > 1.0) cosTheta = 1.0;
                if (cosTheta < -1.0) cosTheta = -1.0;
                sinTheta = Math.Sqrt(1 - Math.Pow(cosTheta, 2));

                if (clusterEntry.avgDirectionVector.y < 0) {
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
                if (clusterEntry.nTrajectories >= m_parameter.minLnsParam) {
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
            m_document.m_clusterRatio = (double)trajectories.Count / (double)m_document.m_trajectoryList.Count;

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
                if (lineSegments.Count >= m_parameter.minLnsParam) {
                    if (Math.Abs(candidatePoint.orderingValue - prevOrderingValue) > ((double)m_parameter.minSegmentLength / 1.414)) {
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
            int nLineSegmentsInSet = lineSegments.Count;
            Point2D clusterPoint = new Point2D(0, 0);
            Point2D sweepPoint = new Point2D(0, 0);

            foreach (int iter in lineSegments) {
                // get the sweep point of each line segment
                // this point is parallel to the current value of the sweeping direction
                sweepPoint = getSweepPointOfLineSegment(clusterEntry, currValue, iter);
                clusterPoint += sweepPoint / (double)nLineSegmentsInSet;
            }

            // NOTE: this program code works only for the 2-dimensional data
            double x = GET_X_REV_ROTATION(clusterPoint.x, clusterPoint.y, clusterEntry.cosTheta, clusterEntry.sinTheta);
            double y = GET_Y_REV_ROTATION(clusterPoint.x, clusterPoint.y, clusterEntry.cosTheta, clusterEntry.sinTheta);

            // register the obtained cluster point (i.e., the average of all the sweep points)
            clusterEntry.clusterPointArray.Add(new Point2D(x, y));

            return;
        }

        private Point2D getSweepPointOfLineSegment(LineSegmentCluster clusterEntry,
                double currValue, int lineSegmentId) {

            Segment lineSegmentPoint = m_lineSegmentPointArray[lineSegmentId];     //  2n-dimensional point

            //  NOTE: this program code works only for the 2-dimensional data
            double newStartX, newEndX, newStartY, newEndY;
            newStartX = GET_X_ROTATION(lineSegmentPoint.start.x, lineSegmentPoint.start.y, clusterEntry.cosTheta, clusterEntry.sinTheta);
            newEndX = GET_X_ROTATION(lineSegmentPoint.end.x, lineSegmentPoint.end.y, clusterEntry.cosTheta, clusterEntry.sinTheta);
            newStartY = GET_Y_ROTATION(lineSegmentPoint.start.x, lineSegmentPoint.start.y, clusterEntry.cosTheta, clusterEntry.sinTheta);
            newEndY = GET_Y_ROTATION(lineSegmentPoint.end.x, lineSegmentPoint.end.y, clusterEntry.cosTheta, clusterEntry.sinTheta);

            double coefficient = (currValue - newStartX) / (newEndX - newStartX);

            return new Point2D(currValue, newStartY + coefficient * (newEndY - newStartY));
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

            Segment aLineSegment = m_lineSegmentPointArray[lineSegmentId];
            double orderingValue1 = GET_X_ROTATION(aLineSegment.start.x,
                    aLineSegment.start.y, clusterEntry.cosTheta, clusterEntry.sinTheta);
            double orderingValue2 = GET_X_ROTATION(aLineSegment.end.x,
                    aLineSegment.end.y, clusterEntry.cosTheta, clusterEntry.sinTheta);

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
        private void computeEPSNeighborhood(Point2D startPoint, Point2D endPoint, double eps, HashSet<int> result) {
            result.Clear();
            for (int j = 0; j < m_nTotalLineSegments; j++) {
                double distance = computeDistanceBetweenTwoLineSegments(startPoint, endPoint, m_lineSegmentPointArray[j].start, m_lineSegmentPointArray[j].end);
                //  if the distance is below the threshold, this line segment belongs to the eps-neighborhood
                if (distance <= eps) result.Add(j);
            }
            return;
        }
        private double computeDistanceBetweenTwoLineSegments(Point2D startPoint1,
                Point2D endPoint1, Point2D startPoint2, Point2D endPoint2) {
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
                Cluster pClusterItem = new Cluster(currClusterId);

                for (int j = 0; j < m_lineSegmentClusters[i].nClusterPoints; j++) {
                    pClusterItem.addPointToArray(m_lineSegmentClusters[i].clusterPointArray[j]);
                }

                pClusterItem.setDensity(m_lineSegmentClusters[i].nTrajectories);

                m_document.m_clusterList.Add(pClusterItem);

                currClusterId++;    //  increase the number of const clusters
                                    //  ... END
            }

            return true;
        }


        private double subComputeDistanceBetweenTwoLineSegments(Point2D startPoint1,
                Point2D endPoint1, Point2D startPoint2, Point2D endPoint2,
                double perpendicularDistance, double parallelDistance,
                double angleDistance) {

            double perDistance1, perDistance2;
            double parDistance1, parDistance2;
            double length1, length2;

            //  the length of the first line segment
            length1 = (startPoint1 - endPoint1).Length();
            //  the length of the second line segment
            length2 = (startPoint2 - endPoint2).Length();

            //  compute the perpendicular distance and the parallel distance
            //  START ...
            if (length1 > length2) {
                perDistance1 = measureDistanceFromPointToLineSegment(startPoint1, endPoint1, startPoint2);
                if (m_coefficient < 0.5) parDistance1 = (startPoint1 - m_projectionPoint).Length();
                else parDistance1 = (endPoint1 - m_projectionPoint).Length();

                perDistance2 = measureDistanceFromPointToLineSegment(startPoint1, endPoint1, endPoint2);
                if (m_coefficient < 0.5) parDistance2 = (startPoint1 - m_projectionPoint).Length();
                else parDistance2 = (endPoint1 - m_projectionPoint).Length();
            } else {
                perDistance1 = measureDistanceFromPointToLineSegment(startPoint2, endPoint2, startPoint1);
                if (m_coefficient < 0.5) parDistance1 = (startPoint2 - m_projectionPoint).Length();
                else parDistance1 = (endPoint2 - m_projectionPoint).Length();

                perDistance2 = measureDistanceFromPointToLineSegment(startPoint2, endPoint2, endPoint1);
                if (m_coefficient < 0.5) parDistance2 = (startPoint2 - m_projectionPoint).Length();
                else parDistance2 = (endPoint2 - m_projectionPoint).Length();

            }

            //  compute the perpendicular distance; take (d1^2 + d2^2) / (d1 + d2)
            if (!(perDistance1 == 0.0 && perDistance2 == 0.0)) {
                perpendicularDistance = (perDistance1 * perDistance1 + perDistance2 * perDistance2) / (perDistance1 + perDistance2);
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

        public bool estimateParameterValue(ref Parameter p) {

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
                    computeEPSNeighborhood(m_lineSegmentPointArray[i].start, m_lineSegmentPointArray[i].end, eps, seeds);
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
            p.minSegmentLength = MIN_LINESEGMENT_LENGTH;
            p.MDLCostAdwantage = MDL_COST_ADWANTAGE;
            return true;
        }

    }

}
