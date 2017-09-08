
#include <jni.h>
#include <string>

extern "C"
JNIEXPORT jstring JNICALL
Java_com_google_android_gms_samples_vision_face_facetracker_FaceTrackerActivity_funSpotDetection(JNIEnv *env, jobject ) {

    // TODO
    std::string s = "hello ramakant";

    return env->NewStringUTF(s.c_str());
}