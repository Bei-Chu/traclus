using System.Collections.Generic;

namespace Traclus
{
    public class Cluster
    {
        private int m_clusterId;        // the identifier of this cluster
        private int m_nTrajectories;    // the minimum number of trajectories belonging to this cluster
        private List<Point2D> m_pointArray;   // the array of the cluster points

        public Cluster()
        {
            m_clusterId = -1;
            m_nTrajectories = 0;
            m_pointArray = new List<Point2D>();
        }

        public Cluster(int id)
        {
            m_clusterId = id;
            m_nTrajectories = 0;
            m_pointArray = new List<Point2D>();
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

        public void addPointToArray(Point2D point)
        {
            m_pointArray.Add(point);
        }

        public List<Point2D> getM_PointArray()
        {
            return m_pointArray;
        }
    }

}
