using System.Collections.Generic;

namespace Traclus
{
    public class Cluster
    {
        private int m_clusterId;        // the identifier of this cluster
        private int m_nDimensions;      // the dimensionality of this cluster
        private int m_nTrajectories;    // the number of trajectories belonging to this cluster
        private int m_nPoints;          // the number of points constituting a cluster 
        private List<CMDPoint> m_pointArray;   // the array of the cluster points

        public Cluster()
        {
            m_clusterId = -1;
            m_nDimensions = 2;
            m_nTrajectories = 0;
            m_nPoints = 0;
            m_pointArray = new List<CMDPoint>();
        }

        public Cluster(int id, int nDimensions)
        {
            m_clusterId = id;
            m_nDimensions = nDimensions;
            m_nTrajectories = 0;
            m_nPoints = 0;
            m_pointArray = new List<CMDPoint>();
        }

        public void setM_clusterId(int clusterId)
        {
            m_clusterId = clusterId;
        }

        public int getM_clusterId()
        {
            return m_clusterId;
        }

        /**
         * set m_nTrajectories --the number of trajectories belonging to this cluster
         * @param density
         */
        public void setDensity(int density)
        {
            m_nTrajectories = density;
        }
        /**
         * get the density -- the number of trajectories belonging to this cluster
         * @return density number
         */
        public int getDensity()
        {
            return m_nTrajectories;
        }

        public void addPointToArray(CMDPoint point)
        {
            m_pointArray.Add(point);
            m_nPoints++;
        }

        public List<CMDPoint> getM_PointArray()
        {
            return m_pointArray;
        }
    }

}