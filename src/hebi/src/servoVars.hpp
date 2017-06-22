    //This header file contains all of the important variables that may need to be changed on the fly

    //AlphaCentroid
    //Limit to the length of the arm when choosing alpha centroid
    float armLengthLimit = 0.58; //it's actually 0.584, but I don't want the arm fully extended
    const int num_centroids = 30; //number of centroids captured

    //arm_config and cylinder_reverse
    //Positioning of the arm in left and right configurations
    float j2_Lconfig = -2.3;
    float j2_Rconfig = 2.3;
    float j1_Lconfig = 0.7;
    float j1_Rconfig = -0.7;

    //cylinder_segmentation
    float zCameraFilter = 0.6; //point cloud filtering in camera frame
    float xCameraFilterMin = -0.2; //point cloud filtering in camera frame
    float xCameraFilterMax = 0.2;
    float minRadiusLim = .010; //min radius of stalk
    float maxRadiusLim = .025; //max radius of stalk
    float distThresh = .005;  //distance thresh within cylinder
    float ransacIterations = 10000;

    //cylinder_servo
    //Lengths of arm links
    float L1 = 0.299;
    float L2 = 0.31; //0.285; //this length is a little short - it pushes the arm into the stalk a bit
    //these two lengths total to 0.584 (see armLengthLimit variable)

    float hyp = L1;
    float hyp_camera = .245; //position of the camera on length2
