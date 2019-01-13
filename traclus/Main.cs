using System;

namespace Traclus {

    class Traclus {

        static void Main(String[] args) {

            if (args.Length == 4) {
                TraClusterDoc tcd = new TraClusterDoc();
                tcd.OpenDocument(args[0]);
                tcd.ClusterGenerate(double.Parse(args[2]), int.Parse(args[3])); // 25, 5~7
                tcd.WriteResult(args[1]);
            } else if (args.Length == 2) {
                TraClusterDoc tcd = new TraClusterDoc();
                tcd.OpenDocument(args[0]);

                Parameter p = tcd.EstimateParameter();
                if (p != null) {
                    Console.WriteLine("Based on the algorithm, the suggested parameters are:\n" + "eps:" + p.epsParam + "  minLns:" + p.minLnsParam);
                }
                tcd.ClusterGenerate(p.epsParam, p.minLnsParam);
                tcd.WriteResult(args[1]);
            } else {
                Console.WriteLine("Please give me 2 or 4 input parameters! \n "
                        + "If you have no idea how to decide eps and minLns, just feed in 2 parameters (inputFilePath, outputFilePath):\n"
                        + "--e.g. traclus.exe deer_1995.tra testOut.txt \n"
                        + "If you know the two parameters, just feed in all the 4 parameters (inputFilePath, outputFilePath, eps, minLns)"
                        + "--e.g. traclus.exe deer_1995.tra testOut.txt 29 8 \n");
            }
        }
    }

}
