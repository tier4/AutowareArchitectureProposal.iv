NDT literature review {#ndt-literature-review}
=============================================

# Introduction

The Normal Distributions Transform (NDT) is a technique which transforms a raw point cloud into a
grid-based representation, where each grid cell is approximated with a multivariate Gaussian
distribution.

The purpose of this representation is to provide a compact form for representing the point cloud.
Doing so also permits the holistic representation of the scan matching, or scan registration problem
as a differentiable optimization problem.

This permits us to use the NDT algorithm to do point cloud-based localization.

# Literature Review

The following is a list of papers relating specifically to the NDT technique for point cloud scan
matching:


- [Scan Registration for Autonomous Mining Vehicles Using 3D-NDT](http://aass.oru.se/Research/mro/publications/2007/Magnusson_etal_2007-JFR-3D_Scan_Registration_for_Autonomous_Mining_Vehicles.pdf)
    - Extends the original NDT paper from 3 DoF scan registration to a full 6 DoF registration
    - This can be thought of as Point-to-Distribution NDT (P2D)
- [The Three-Dimensional Normal-Distributions Transform – an Efficient Representation for Registration, Surface Analysis, and Loop Detection](http://www.diva-portal.org/smash/get/diva2:276162/FULLTEXT02.pdf)
    - Thesis for NDT, covers 2D-NDT, 3D-NDT, Color-NDT
    - Also covers variations on NDT representation:
        - Trilinear interpolation
        - Linked nodes, etc.
    - Has a variety of empirical results:
        - NDT is faster and more consistent than ICP
        - NDT is somewhat more robust than ICP, but still needs good initializations
        - Color-NDT is better than plain NDT
    - Also covers applications, e.g. loop-closure
- [Fast and accurate scan registration through minimization of the distance between compact 3D NDT representations](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.817.5962&rep=rep1&type=pdf)
    - Introduces Distribution-to-Distribution NDT (D2D)
    - Faster than P2D-NDT
- [Beyond Points: Evaluating Recent 3D Scan-Matching Algorithms](http://spencer.eu/papers/MagnussonICRA2015.pdf)
    - Compares multiple NDT techniques, and various tweaks on the algorithm, i.e.:
        - (Trilinear) interpolation for handling boundary effects
        - Iterative subdivision to improve robustness
        - Linked nodes for handling outliers/missing data
    - P2D is more robust for unstructured environments and small overlaps between scans compared to D2D
    - D2D is faster and less sensitive to poor initial estimates
- [Point cloud registration from local feature correspondences—Evaluation on challenging datasets](https://journals.plos.org/plosone/article?id=10.1371/journal.pone.0187943#pone-0187943-t002)
    - Compares ICP variants and NDT
    - NDT is good at handling translation, and among the fastest of techniques
    - NDT is so-so at handling rotations
- [Evaluation of 3D Registration Reliability and Speed – A Comparison of ICP and NDT](https://www.researchgate.net/profile/Achim_Lilienthal/publication/224557318_Evaluation_of_3D_Registration_Reliability_and_Speed_-_A_Comparison_of_ICP_and_NDT/links/0912f508b2e8c8dc02000000/Evaluation-of-3D-Registration-Reliability-and-Speed-A-Comparison-of-ICP-and-NDT.pdf)
    - Compares ICP with NDT variants
    - Introduces trilinear interpolation for NDT - results in more robustness
    - NDT is slightly faster than ICP, generally more robust
- [Incorporating Ego-motion Uncertainty Estimates in Range Data Registration ](http://iliad-project.eu/wp-content/uploads/papers/IROS17_1012_FI.pdf)
    - Adds a motion constraint to the optimization objective
    - Slower than normal D2D
    - However, much more performant on low-information environments (i.e. endless hallway)
- [Semantic-assisted 3D Normal Distributions Transform for scan registration in environments with limited structure](http://eprints.lincoln.ac.uk/28481/1/iros_se_ndt.pdf)
    - Generate several NDTs based on metric/feature-based partitions (which also discard outliers),
    extend optimization objective to handle these multiple NDTs
    - Shows strong speed and accuracy results
