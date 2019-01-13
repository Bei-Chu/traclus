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

        public double m_clusterRatio; // for debugging
        public List<Trajectory> m_trajectoryList; // input for traclus
        public List<Cluster> m_clusterList; // output of traclus
        private Parameter m_parameter;

        public TraClusterDoc() {

            m_clusterRatio = 0.0;
            m_trajectoryList = new List<Trajectory>();
            m_clusterList = new List<Cluster>();
            m_parameter = new Parameter();
            m_parameter.epsParam = 0.0;
            m_parameter.minLnsParam = 0;
        }

        public bool OpenDocument(String inputFileName) {
            TextReader reader = null;
            try {
			    reader = File.OpenText(inputFileName);

                int nDimensions = int.Parse(reader.ReadLine());
                if (nDimensions != 2) {
                    throw new NotImplementedException("Dimension must be 2");
                }

                int nTrajectories = int.Parse(reader.ReadLine());

                // the trajectory Id, the number of points, the coordinate of a point ...
                for (int i = 0; i < nTrajectories; i++) {

                    String str = reader.ReadLine();

                    Scanner sc = new Scanner(str);

                    int trajectoryId = sc.nextInt(); //trajectoryID
                    int nPoints = sc.nextInt(); // number of points in the trajectory

                    Trajectory pTrajectoryItem = new Trajectory(trajectoryId);
                    for (int j = 0; j < nPoints; j++) {
                        Point2D point = new Point2D(0, 0);   // initialize the CMDPoint class for each point

                        point.x = sc.nextDouble();
                        point.y = sc.nextDouble();
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

        public bool ClusterGenerate(double epsParam, int minLnsParam) {

            m_parameter.epsParam = epsParam;
            m_parameter.minLnsParam = minLnsParam;

            ClusterGen generator = new ClusterGen(this);

            if (m_trajectoryList.Count == 0) {
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

                    double x = m_clusterList[i].getM_PointArray()[j].x;
                    double y = m_clusterList[i].getM_PointArray()[j].y;
                    Console.Write("   " + x + " " + y + "   ");
                }
                Console.WriteLine("");
            }

            return true;
        }

        public bool WriteResult(String clusterFileName) {

            TextWriter writer = null;
            try {
                writer = File.CreateText(clusterFileName);

                writer.WriteLine("epsParam:" + m_parameter.epsParam + "   minLnsParam:" + m_parameter.minLnsParam);

                for (int i = 0; i < m_clusterList.Count; i++) {
                    // m_clusterList.
                    writer.WriteLine("clusterID: " + m_clusterList[i].getM_clusterId() + "  Points Number:  " + m_clusterList[i].getM_PointArray().Count);
                    for (int j = 0; j < m_clusterList[i].getM_PointArray().Count; j++) {

                        double x = m_clusterList[i].getM_PointArray()[j].x;
                        double y = m_clusterList[i].getM_PointArray()[j].y;
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

        public Parameter EstimateParameter() {
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
