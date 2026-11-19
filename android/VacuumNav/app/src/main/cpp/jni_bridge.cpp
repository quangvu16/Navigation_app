// jni_bridge.cpp — thin JNI surface over vacuum_core's simulator.
//
// Deliberately dumb: no state beyond the SimRobot instance behind a raw
// handle. All policy (rates, safety, config) lives in Kotlin/c++, never here.
// The Phase-2+ engine will replace SimRobot with the real NativeEngine while
// keeping these entry points (init/step/pose/scan), so the service code does
// not care whether the brain is simulated or real.

#include <jni.h>

#include <memory>
#include <mutex>
#include <vector>

#include "sim/sim_robot.h"
#include "vacuum/robot_config.h"

namespace {

using vacuum::Pose2D;
using vacuum::RobotConfig;
using vacuum::SimNoise;
using vacuum::SimRobot;
using vacuum::SimWorld;
using vacuum::FlatJson;

struct Engine {
  std::mutex mutex;
  std::unique_ptr<SimRobot> robot;
};

Engine* fromHandle(jlong handle) {
  return reinterpret_cast<Engine*>(handle);
}

std::string jstringToString(JNIEnv* env, jstring js) {
  const char* chars = env->GetStringUTFChars(js, nullptr);
  std::string out(chars ? chars : "");
  if (chars) env->ReleaseStringUTFChars(js, chars);
  return out;
}

}  // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeCreate(
    JNIEnv* env, jobject, jstring configJson, jbyteArray worldCells,
    jint worldW, jint worldH, jdouble worldRes, jdouble originX,
    jdouble originY, jlong seed) {
  const RobotConfig cfg = RobotConfig::fromJson(FlatJson::parse(jstringToString(env, configJson)));

  SimWorld world;
  if (worldCells != nullptr && worldW > 0 && worldH > 0) {
    const jsize n = env->GetArrayLength(worldCells);
    std::vector<uint8_t> cells(static_cast<size_t>(n));
    env->GetByteArrayRegion(worldCells, 0, n,
                            reinterpret_cast<jbyte*>(cells.data()));
    world = SimWorld(worldW, worldH, worldRes, vacuum::Vec2(originX, originY),
                     cells.data());
  } else {
    // Default demo scene: a 4 m empty room. The app will load real floor
    // plans (PNG -> grid) once the Maps screen exists (Phase 3).
    world = SimWorld::emptyRoom(4.0, 0.05, 0.10);
  }

  auto* engine = new Engine();
  // Realistic noisy physics in the app; tests use the host build with quiet
  // noise for exactness. A later Settings switch can expose the seed.
  engine->robot = std::make_unique<SimRobot>(cfg, std::move(world), SimNoise(),
                                             static_cast<uint32_t>(seed));
  return reinterpret_cast<jlong>(engine);
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeDestroy(JNIEnv*, jobject,
                                                     jlong handle) {
  delete fromHandle(handle);
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeSetWheelRpm(JNIEnv*, jobject,
                                                         jlong handle,
                                                         jfloat rpmL,
                                                         jfloat rpmR) {
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    e->robot->setWheelRpm(rpmL, rpmR);
  }
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeStep(JNIEnv*, jobject,
                                                  jlong handle, jdouble dt) {
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    e->robot->step(dt);
  }
}

// pose: [x, y, theta] true pose; odom: [x, y, theta] tick-derived pose.
JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeGetPose(JNIEnv* env, jobject,
                                                     jlong handle,
                                                     jfloatArray out3) {
  float buf[3] = {0, 0, 0};
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    const Pose2D& p = e->robot->sensors().truePose;
    buf[0] = static_cast<float>(p.x);
    buf[1] = static_cast<float>(p.y);
    buf[2] = static_cast<float>(p.theta);
  }
  env->SetFloatArrayRegion(out3, 0, 3, buf);
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeGetOdom(JNIEnv* env, jobject,
                                                     jlong handle,
                                                     jfloatArray out3) {
  float buf[3] = {0, 0, 0};
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    const Pose2D& p = e->robot->sensors().odomPose;
    buf[0] = static_cast<float>(p.x);
    buf[1] = static_cast<float>(p.y);
    buf[2] = static_cast<float>(p.theta);
  }
  env->SetFloatArrayRegion(out3, 0, 3, buf);
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeGetScanRanges(JNIEnv* env,
                                                           jobject,
                                                           jlong handle,
                                                           jfloatArray out) {
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    const auto& ranges = e->robot->sensors().ranges;
    env->SetFloatArrayRegion(out, 0,
                             static_cast<jsize>(ranges.size()), ranges.data());
  }
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeGetScanIntensity(JNIEnv* env,
                                                              jobject,
                                                              jlong handle,
                                                              jbyteArray out) {
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    const auto& inten = e->robot->sensors().intensity;
    env->SetByteArrayRegion(out, 0, static_cast<jsize>(inten.size()),
                            reinterpret_cast<const jbyte*>(inten.data()));
  }
}

// meta: [scanId, startS, endS, rotationHz]
JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeGetScanMeta(JNIEnv* env, jobject,
                                                         jlong handle,
                                                         jfloatArray out4) {
  float buf[4] = {0, 0, 0, 6.0f};
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    const auto& s = e->robot->sensors();
    buf[0] = static_cast<float>(s.scanId);
    buf[1] = static_cast<float>(s.scanStartS);
    buf[2] = static_cast<float>(s.scanEndS);
    buf[3] = static_cast<float>(s.rotationHz);
  }
  env->SetFloatArrayRegion(out4, 0, 4, buf);
}

// out: [front, left, right] metres, 0 = no echo (HBT sonar convention).
JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeGetSonar(JNIEnv* env, jobject,
                                                      jlong handle,
                                                      jfloatArray out3) {
  float buf[3] = {0, 0, 0};
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    buf[0] = static_cast<float>(e->robot->sensors().sonarM[0]);
    buf[1] = static_cast<float>(e->robot->sensors().sonarM[1]);
    buf[2] = static_cast<float>(e->robot->sensors().sonarM[2]);
  }
  env->SetFloatArrayRegion(out3, 0, 3, buf);
}

// out: [gyroZ rad/s, batteryV, rpmL, rpmR, ticksL, ticksR]
JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NativeEngine_nativeGetStatus(JNIEnv* env, jobject,
                                                       jlong handle,
                                                       jfloatArray out6) {
  float buf[6] = {0, 0, 0, 0, 0, 0};
  if (Engine* e = fromHandle(handle)) {
    std::lock_guard<std::mutex> lock(e->mutex);
    const auto& s = e->robot->sensors();
    buf[0] = static_cast<float>(s.gyroZ);
    buf[1] = static_cast<float>(s.batteryV);
    buf[2] = static_cast<float>(s.rpmL);
    buf[3] = static_cast<float>(s.rpmR);
    buf[4] = static_cast<float>(s.ticksL);
    buf[5] = static_cast<float>(s.ticksR);
  }
  env->SetFloatArrayRegion(out6, 0, 6, buf);
}

}  // extern "C"
