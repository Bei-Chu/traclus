using System;
using System.Collections.Generic;
using System.IO;

namespace Traclus {

    public class Parameter
    {
        public double epsParam;
        public int minLnsParam;
    }

    public class TraClusterDoc {

        public int m_nDimensions;
        public int m_nTrajectories;
        public int m_nClusters;
        public double m_clusterRatio;
        public int m_maxNPoints;
        public List<Trajectory> m_trajectoryList;
        public List<Cluster> m_clusterList;

        public TraClusterDoc() {

            m_nTrajectories = 0;
            m_nClusters = 0;
            m_clusterRatio = 0.0;
            m_trajectoryList = new List<Trajectory>();
            m_clusterList = new List<Cluster>();
        }

        public bool onOpenDocument(String inputFileName) {
            TextReader reader = null;
            try {
			    reader = File.OpenText(inputFileName);

                m_nDimensions = int.Parse(reader.ReadLine());
                m_nTrajectories = int.Parse(reader.ReadLine());

                m_maxNPoints = -1; // initialize for comparison

                // the trajectory Id, the number of points, the coordinate of a point ...
                for (int i = 0; i < m_nTrajectories; i++) {

                    String str = reader.ReadLine();

                    Scanner sc = new Scanner(str);

                    int trajectoryId = sc.nextInt(); //trajectoryID
                    int nPoints = sc.nextInt(); // number of points in the trajectory

                    if (nPoints > m_maxNPoints) {
                        m_maxNPoints = nPoints;
                    }

                    Trajectory pTrajectoryItem = new Trajectory(trajectoryId, m_nDimensions);
                    for (int j = 0; j < nPoints; j++) {
                        CMDPoint point = new CMDPoint(m_nDimensions);   // initialize the CMDPoint class for each point

                        for (int k = 0; k < m_nDimensions; k++) {
                            double value = sc.nextDouble();
                            point.setM_coordinate(k, value);
                        }
                        pTrajectoryItem.addPointToArray(point);
                    }

                    m_trajectoryList.Add(pTrajectoryItem);
                }
            } catch (FileNotFoundException e) {
                Console.WriteLine(e);
                Console.WriteLine("Unable to open input file");
            } catch (FormatException e) {
                Console.WriteLine(e);
            } catch (IOException e) {
                Console.WriteLine(e);
            } finally {
                try {
                    if (reader != null) reader.Close();
                } catch (IOException e) {
                    Console.WriteLine(e);
                }
            }

            return true;
        }

        public bool onClusterGenerate(String clusterFileName, double epsParam, int minLnsParam) {
            //////////////////////////////////////////////////still to be written

            ClusterGen generator = new ClusterGen(this);

            if (m_nTrajectories == 0) {
                Console.WriteLine("Load a trajectory data set first");
            }

            // FIRST STEP: Trajectory Partitioning
            if (!generator.partitionTrajectory())
            {
                Console.WriteLine("Unable to partition a trajectory\n");
                return false;
            }

            // SECOND STEP: Density-based Clustering
            if (!generator.performDBSCAN(epsParam, minLnsParam))
            {
                Console.WriteLine("Unable to perform the DBSCAN algorithm\n");
                return false;
            }

            // THIRD STEP: Cluster Construction
            if (!generator.constructCluster())
            {
                Console.WriteLine("Unable to construct a cluster\n");
                return false;
            }


            for (int i = 0; i < m_clusterList.Count; i++) {
                //m_clusterList.
                Console.WriteLine(m_clusterList[i].getM_clusterId());
                for (int j = 0; j < m_clusterList[i].getM_PointArray().Count; j++) {

                    double x = m_clusterList[i].getM_PointArray()[j].getM_coordinate(0);
                    double y = m_clusterList[i].getM_PointArray()[j].getM_coordinate(1);
                    Console.Write("   " + x + " " + y + "   ");
                }
                Console.WriteLine("");
            }

            TextWriter writer = null;
            try {
                writer = File.CreateText(clusterFileName);

                writer.WriteLine("epsParam:" + epsParam + "   minLnsParam:" + minLnsParam);

                for (int i = 0; i < m_clusterList.Count; i++) {
                    // m_clusterList.
                    writer.WriteLine("clusterID: " + m_clusterList[i].getM_clusterId() + "  Points Number:  " + m_clusterList[i].getM_PointArray().Count);
                    for (int j = 0; j < m_clusterList[i].getM_PointArray().Count; j++) {

                        double x = m_clusterList[i].getM_PointArray()[j].getM_coordinate(0);
                        double y = m_clusterList[i].getM_PointArray()[j].getM_coordinate(1);
                        writer.Write(x + " " + y + "   ");
                    }
                    writer.Write("\n");
                }

            } catch (FileNotFoundException e) {
                Console.WriteLine(e);
            } catch (IOException e) {
                Console.WriteLine(e);
            } finally {
                try {
                    if (writer != null) writer.Close();
                } catch (IOException e) {
                    Console.WriteLine(e);
                }
            }
            return true;
        }

        public Parameter onEstimateParameter() {
            Parameter p = new Parameter();
            ClusterGen generator = new ClusterGen(this);
            if (!generator.partitionTrajectory()) {
                Console.WriteLine("Unable to partition a trajectory\n");
                return null;
            }
            if (!generator.estimateParameterValue(p)) {
                Console.WriteLine("Unable to calculate the entropy\n");
                return null;
            }
            return p;
        }

    }

}
