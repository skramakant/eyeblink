#include <jni.h>
#include <string>
#include <opencv2/opencv.hpp>
#include <android/log.h>

using namespace cv;
using namespace std;


extern "C"
JNIEXPORT jint JNICALL
Java_com_google_android_gms_samples_vision_face_facetracker_FaceTrackerActivity_checkForSpoofing(JNIEnv *env, jobject instance,
                                                           jlong mRgbA,jlong mRgbG) {

    Mat& mReal = *(Mat*) mRgbA;
    Mat& mG = *(Mat*) mRgbG;

    Mat gray = mReal.clone();
    cvtColor(gray,gray,CV_RGB2GRAY);

    Mat g1, g2;
    GaussianBlur(gray, g1, Size(3, 3),0,0, 0);
    GaussianBlur(gray, g2, Size(1, 1),0,0, 0);
    Mat result1 = g1 - g2;

    Scalar scalar = sum(result1);
    double value = scalar[0];

    ostringstream strs;
    strs << value;
    string str = strs.str();
    putText(mReal, str, Point2f(40,220), FONT_HERSHEY_SIMPLEX, 1,  Scalar(255));
    __android_log_print(ANDROID_LOG_VERBOSE,"EyeValue", "ramakant float = %f",value);
    if(value < 35555.0){//350000.0
        putText(mReal, "Spoofing", Point2f(40,120), FONT_HERSHEY_SIMPLEX, 1,  Scalar(255));
        return 1;
    }

    cout << "Sum "<< sum(result1)<< endl;
    return 0;
}



extern "C"
JNIEXPORT void JNICALL

Java_com_google_android_gms_samples_vision_face_facetracker_FaceTrackerActivity_thresh_callback(Mat src_gray,int, void* )
{
    Mat canny_output;
    int thresh = 100;
    int max_thresh = 255;
    RNG rng(12345);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using canny
    Canny( src_gray, canny_output, thresh, thresh*2, 3 );
    /// Find contours
    findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

    /// Draw contours
    Mat drawing = Mat::zeros( canny_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        drawContours( drawing, contours, i, color, 2, 8, hierarchy, 0, Point() );
    }

    /// Show in a window
    //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    //imshow( "Contours", drawing );
}

extern "C"
JNIEXPORT jint JNICALL
Java_com_google_android_gms_samples_vision_face_facetracker_FaceTrackerActivity_findBrightSpot(JNIEnv *env, jobject instance,jlong mRgbA,
                                                          jlong mRgbG,jlong mRgbT) {

    Mat& mR = *(Mat*) mRgbA;
    Mat& mG = *(Mat*) mRgbG;
    Mat& mT = *(Mat*) mRgbT;
    //cvThreshold(mRgbG,mT,200,255,CV_THRESH_BINARY);
    GaussianBlur(mG , mG, cv::Size(0, 0), 3, 3 );
    threshold(mG,mT,250,255,CV_THRESH_BINARY);

//    //morphological opening (remove small objects from the foreground)
//    erode(mT, mT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
//    dilate( mT, mT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
//
//    //morphological closing (fill small holes in the foreground)
//    dilate( mT, mT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
//    erode(mT, mT, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

    //Mat& grayscale = *(Mat*)mRgbT;
    //std::string  returnValue = "true";
//    IplImage* grayscale = (IplImage *) mRgbT;
//    IplImage* img_bw = cvCreateImage( cvGetSize( grayscale ),
//                                      IPL_DEPTH_8U,
//                                      1 );

    // Use thresholding to convert grayscale image
    // into binary
//    cvThreshold( mRgbG,             // source image
//                 mRgbT,                // destination image
//                 200,                    // threhold val.
//                 255,                   // max. val
//                 CV_THRESH_BINARY );    // binary


    // Create IplImage struct for inverted black
    // and white image
    //IplImage* img_bw_inv = cvCloneImage( img_bw );
    //IplImage* img_bw_cpy = cvCloneImage( img_bw );

    // Find connected components using OpenCV
//    CvSeq* seq;
//    CvMemStorage* storage = cvCreateMemStorage( 0 );
//
//    cvClearMemStorage( storage );
//      int num_blobs;
//    IplImage tmp = mT;
//
//    num_blobs = cvFindContours( &tmp,
//                                storage,
//                                &seq,
//            sizeof( CvContour ),
//            CV_RETR_LIST,
//            CV_CHAIN_APPROX_NONE,
//            cvPoint( 0, 0 ) );


    //Java_eyeblink_eyeblinkopencv_MainActivity_thresh_callback(mG,0,0);

    //find blob

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    params.filterByColor =true;
    params.blobColor=255;


//// Change thresholds
//    params.thresholdStep = 5;
//    params.minThreshold = 230;
//    params.maxThreshold = 255;

    //Filter by Area.
//    params.filterByArea = true;
//    params.minArea = 150;
    //params.maxArea = 100;
//
//// Filter by Circularity
    params.filterByCircularity = true;
//    params.minCircularity = 0.8;
//    params.maxCircularity = 1.0;

//// Filter by Convexity
    //params.filterByConvexity = true;
//    params.minConvexity = 0.87;

//// Filter by Inertia
    //params.filterByInertia = true;
//    params.minInertiaRatio = 0.01;



// Set up the detector with default parameters.
    //SimpleBlobDetector detector;
    Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);

// Detect blobs.
    std::vector<KeyPoint> keypoints;
    detector->detect( mT, keypoints);
    //__android_log_print(ANDROID_LOG_VERBOSE, "EyeblinkOpencv", "ramakant size = %f",keypoints.max_size() );
    for(int i=0;i<keypoints.size();i++){
        float a = keypoints[0].size;
        __android_log_print(ANDROID_LOG_VERBOSE,"EyeblinkOpencv", "ramakant float = %f",a);
        if(a > 25.0)
            putText(mR, "Spoofing", Point2f(40,120), FONT_HERSHEY_SIMPLEX, 4,  Scalar(255));
    }
    //putText(mT, "Spoofing", Point2f(20,40), FONT_HERSHEY_SIMPLEX, 2,  Scalar(255));
// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    //drawKeypoints( mT, keypoints, mT, Scalar(255,0,0), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

    return 0;
}

extern "C"
JNIEXPORT jstring JNICALL
Java_eyeblink_eyeblinkopencv_MainActivity_stringFromJNI(
        JNIEnv *env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
