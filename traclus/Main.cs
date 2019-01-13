using System;

namespace Traclus {

    class Traclus {

        static void Main(String[] args) {

            if (args.Length == 4) {
                TraClusterDoc tcd = new TraClusterDoc();
                tcd.onOpenDocument(args[0]);
                tcd.onClusterGenerate(args[1], double.Parse(args[2]), int.Parse(args[3])); // 25, 5~7
            } else if (args.Length == 2) {
                TraClusterDoc tcd = new TraClusterDoc();
                tcd.onOpenDocument(args[0]);

                Parameter p = tcd.onEstimateParameter();
                if (p != null) {
                    Console.WriteLine("Based on the algorithm, the suggested parameters are:\n" + "eps:" + p.epsParam + "  minLns:" + p.minLnsParam);
                }
                tcd.onClusterGenerate(args[1], p.epsParam, p.minLnsParam);
            } else {
                Console.WriteLine("Please give me 2 or 4 input parameters! \n "
                        + "If you have no idea how to decide eps and minLns, just feed in 2 parameters (inputFilePath, outputFilePath):\n"
                        + "--e.g. java boliu.Main deer_1995.tra testOut.txt \n"
                        + "If you know the two parameters, just feed in all the 4 parameters (inputFilePath, outputFilePath, eps, minLns)"
                        + "--e.g. java boliu.Main deer_1995.tra testOut.txt 29 8 \n");
            }
        }
    }

}
