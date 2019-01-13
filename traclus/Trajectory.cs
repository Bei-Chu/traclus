using System.Collections.Generic;

namespace Traclus
{
    public class Trajectory
    {

        private int m_trajectoryId; // the identifier of this trajectory
        private List<Point2D> m_pointArray; // the array of the trajectory points
        private List<Point2D> m_partitionPointArray; // the array of the partition points


        public Trajectory()
        {
            m_trajectoryId = -1;
            m_pointArray = new List<Point2D>();
            m_partitionPointArray = new List<Point2D>();

        }

        public Trajectory(int id)
        {
            m_trajectoryId = id;
            m_pointArray = new List<Point2D>();
            m_partitionPointArray = new List<Point2D>();
        }
        //two methods	
        public void addPointToArray(Point2D point)
        {
            m_pointArray.Add(point);
        }

        public void addPartitionPointToArray(Point2D point)
        {
            m_partitionPointArray.Add(point);
        }

        public void setM_trajectoryId(int id)
        {
            m_trajectoryId = id;
        }

        public int getM_trajectoryId()
        {
            return m_trajectoryId;
        }

        public List<Point2D> getM_pointArray()
        {
            return m_pointArray;
        }

        public void setM_pointArray(List<Point2D> pointArray)
        {
            m_pointArray = pointArray;
        }

        public List<Point2D> getM_partitionPointArray()
        {
            return m_partitionPointArray;
        }

        public void setM_partitionPointArray(List<Point2D> partitionPointArray)
        {
            m_partitionPointArray = partitionPointArray;
        }



    }

}
