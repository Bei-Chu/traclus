# TraClus Algorithm

This is an implementation of TraClus (TRAjectory CLUStering) [1] algorithm in C#, ported from the JAVA implementation https://github.com/luborliu/TraClusAlgorithm.


# Input file format


The data format is as follows:

    1st line: Number of dimensions (2 for example)

    2nd line: Number of trajectories (32 for example)

    3rd line: Trajectory Index (starting from 0, so 0 for this line), Number of trajectory points in this trajectory (n for example), X1, Y1, X2, Y2, …., Xn, Yn

    4th line: Trajectory Index (starting from 0, so 1 for this line), Number of trajectory points in this trajectory (m for example), X1, Y1, X2, Y2, …., Xm, Ym
    .
    .
    .
    34th line:Trajectory Index (starting from 0, so 31 for this line), Number of trajectory points in this trajectory (j for example), X1, Y1, X2, Y2, …., Xj, Yj



# Reference


[1] Lee, Jae-Gil, Jiawei Han, and Kyu-Young Whang. "Trajectory clustering: a partition-and-group framework."
Proceedings of the 2007 ACM SIGMOD international conference on Management of data. ACM, 2007.

