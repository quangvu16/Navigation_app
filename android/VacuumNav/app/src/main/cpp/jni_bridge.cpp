// jni_bridge.cpp — thin JNI surface over vacuum_core's simulator.
//
// Deliberately dumb: no state beyond the SimRobot instance behind a raw
// handle. All policy (rates, safety, config) lives in Kotlin/c++, never here.
// The Phase-2+ engine will replace SimRobot with the real NativeEngine while
// keeping these entry points (init/step/pose/scan), so the service code does
// not care whether the brain is simulated or real.

#include <jni.h>

#include <cstring>
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
  // Start at the world centre: the default room's centre is (2, 2) and a
  // real floor plan's centre is almost always free space. Starting at
  // (0, 0) put the robot inside the wall — half the scans returned nothing.
  // (Phase 3's "set pose" screen will override this.)
  const vacuum::Vec2 centre(
      world.origin().x + world.width() * world.resolution() * 0.5,
      world.origin().y + world.height() * world.resolution() * 0.5);
  engine->robot = std::make_unique<SimRobot>(cfg, std::move(world), SimNoise(),
                                             static_cast<uint32_t>(seed));
  engine->robot->setPose(Pose2D(centre.x, centre.y, 0.0));
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

}  // extern "C" (sim Engine ends; Eigen-backed mapper code must be C++)

// ---------------------------------------------------------------------------
// MapperEngine — Phase 3 live mapping. Own handle (independent of the sim
// Engine): preprocess -> correlative match -> integrate -> keyframes/loops.
// The Kotlin side calls feed() from the scan collector and snapshot() when
// the version changes (prompt §6: build the bitmap only on change).
// ---------------------------------------------------------------------------

#include "vacuum/mapper.h"
#include "vacuum/scan_matcher.h"

namespace {

struct MapperHolder {
  std::mutex mutex;
  RobotConfig cfg;
  vacuum::ScanPreprocessor pre;
  vacuum::Mapper mapper;
  uint64_t version = 0;   // bumped whenever keyframes change

  explicit MapperHolder(const RobotConfig& c) : cfg(c) { pre.cfg = c; }
};

MapperHolder* fromMapper(jlong h) {
  return reinterpret_cast<MapperHolder*>(h);
}

}  // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_vacuum_nav_robot_MapperEngine_nativeCreate(JNIEnv* env, jobject,
                                                    jstring configJson) {
  const RobotConfig cfg =
      RobotConfig::fromJson(FlatJson::parse(jstringToString(env, configJson)));
  return reinterpret_cast<jlong>(new MapperHolder(cfg));
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_MapperEngine_nativeDestroy(JNIEnv*, jobject,
                                                     jlong handle) {
  delete fromMapper(handle);
}

// Feed one scan with the pose guess (map frame). Writes the matched pose
// into outPose[3]. De-skew uses the same pose for start/end here; LDS2
// start/end times land with the real link's deskew (Phase 3 hardware).
JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_MapperEngine_nativeFeed(
    JNIEnv* env, jobject, jlong handle, jfloatArray ranges, jint bins,
    jfloat poseX, jfloat poseY, jfloat poseTheta, jfloatArray outPose) {
  MapperHolder* m = fromMapper(handle);
  if (!m) return;
  std::lock_guard<std::mutex> lock(m->mutex);

  std::vector<float> r(static_cast<size_t>(bins));
  env->GetFloatArrayRegion(ranges, 0, bins, r.data());

  const Pose2D guess(poseX, poseY, poseTheta);
  // Preprocess (range gate + voxel downsample; de-skew is a no-op when
  // start == end pose), then match/integrate/keyframe/loop inside Mapper.
  const auto pts = m->pre.process(r.data(), bins, guess, guess);
  const Pose2D matched = m->mapper.process(guess, pts);
  m->version = m->mapper.keyframes().size();

  const float out[3] = {static_cast<float>(matched.x),
                        static_cast<float>(matched.y),
                        static_cast<float>(matched.theta)};
  env->SetFloatArrayRegion(outPose, 0, 3, out);
}

// Monotonic map version (keyframe count): the UI rebuilds its bitmap when
// this changes and nothing else (prompt §6).
JNIEXPORT jint JNICALL
Java_com_vacuum_nav_robot_MapperEngine_nativeVersion(JNIEnv*, jobject,
                                                     jlong handle) {
  MapperHolder* m = fromMapper(handle);
  if (!m) return 0;
  std::lock_guard<std::mutex> lock(m->mutex);
  return static_cast<jint>(m->version);
}

// Occupancy snapshot: [w i32][h i32][gx0 i32][gy0 i32][res f32] + w*h
// cells (0 free, 1 occupied, 2 unknown), row 0 = bottom of the world.
JNIEXPORT jbyteArray JNICALL
Java_com_vacuum_nav_robot_MapperEngine_nativeSnapshot(JNIEnv* env, jobject,
                                                      jlong handle) {
  MapperHolder* m = fromMapper(handle);
  if (!m) return nullptr;
  std::lock_guard<std::mutex> lock(m->mutex);

  const auto& grid = m->mapper.map();
  const int w = grid.width(), h = grid.height();
  const size_t header = 20;
  const size_t total = header + static_cast<size_t>(w) * h;
  std::vector<uint8_t> buf(total);
  auto putI32 = [&](size_t off, int32_t v) {
    for (int i = 0; i < 4; ++i)
      buf[off + i] = static_cast<uint8_t>((v >> (8 * i)) & 0xFF);
  };
  putI32(0, w);
  putI32(4, h);
  putI32(8, grid.originX());
  putI32(12, grid.originY());
  float res = static_cast<float>(grid.resolution());
  std::memcpy(&buf[16], &res, 4);

  for (int y = 0; y < h; ++y) {
    for (int x = 0; x < w; ++x) {
      const int gx = grid.originX() + x, gy = grid.originY() + y;
      uint8_t v;   // 0 free, 1 occupied, 2 unknown
      if (grid.occupied(gx, gy)) v = 1;
      else if (grid.free(gx, gy)) v = 0;
      else v = 2;
      buf[header + static_cast<size_t>(y) * w + x] = v;
    }
  }

  jbyteArray out = env->NewByteArray(static_cast<jsize>(total));
  env->SetByteArrayRegion(out, 0, static_cast<jsize>(total),
                          reinterpret_cast<const jbyte*>(buf.data()));
  return out;
}

}  // extern "C" (mapper ends; Eigen-backed nav code must be C++)

// ---------------------------------------------------------------------------
// NavEngine - Phase 4 go-to-goal: layered costmap (static from the mapper's
// snapshot + keepout zones + decaying scan obstacles) + GoToController
// (A* / RPP / recovery). One handle; Kotlin drives it at 20 Hz from the
// service control thread.
// ---------------------------------------------------------------------------

#include "vacuum/costmap.h"
#include "vacuum/frontier.h"
#include "vacuum/goto_controller.h"

namespace {

struct NavHolder {
  std::mutex mutex;
  RobotConfig cfg;
  vacuum::OccupancyGrid grid;   // retained for frontier queries (Phase 6)
  vacuum::Costmap cm;
  vacuum::GoToController gc;
  bool hasMap = false;
  long nowMs = 0;           // virtual clock driven by step() (50 ms ticks)
  double obstClockS = 0.0;  // seconds, for the2 s obstacle decay

  // Parse the GridSnapshot format (same as nativeSnapshot) into a fresh
  // OccupancyGrid and install it as the costmap's static layer.
  bool setMap(const uint8_t* d, size_t n) {
    if (n < 20) return false;
    auto i32 = [&](size_t off) {
      return static_cast<int32_t>(
          static_cast<uint32_t>(d[off]) |
          (static_cast<uint32_t>(d[off + 1]) << 8) |
          (static_cast<uint32_t>(d[off + 2]) << 16) |
          (static_cast<uint32_t>(d[off + 3]) << 24));
    };
    const int w = i32(0), h = i32(4), gx0 = i32(8), gy0 = i32(12);
    float res;
    std::memcpy(&res, d + 16, 4);
    if (w <= 0 || h <= 0 || res <= 0 || n < 20 + static_cast<size_t>(w) * h)
      return false;
    vacuum::OccupancyGrid fresh(res);
    // Grow the grid to cover (gx0, gy0, w, h) by writing the corners first.
    fresh.setLogOdds(gx0, gy0, -1.0f);
    fresh.setLogOdds(gx0 + w - 1, gy0 + h - 1, -1.0f);
    for (int y = 0; y < h; ++y)
      for (int x = 0; x < w; ++x) {
        const uint8_t c = d[20 + static_cast<size_t>(y) * w + x];
        // 0 free, 1 occupied, 2 unknown.
        fresh.setLogOdds(gx0 + x, gy0 + y,
                         c == 1 ? vacuum::OccupancyGrid::kLOddMax
                                : c == 0 ? -1.0f : 0.0f);
      }
    grid = fresh;   // keep for frontier exploration (Phase 6)
    cm.setStatic(fresh);
    hasMap = true;
    return true;
  }
};

NavHolder* fromNav(jlong h) { return reinterpret_cast<NavHolder*>(h); }

int stateCode(vacuum::GoToState s) {
  using G = vacuum::GoToState;
  switch (s) {
    case G::IDLE: return 0;
    case G::REJECTED: return 1;
    case G::PLANNING: return 2;
    case G::FOLLOWING: return 3;
    case G::RECOVERING: return 4;
    case G::REACHED: return 5;
    case G::STUCK: return 6;
  }
  return 0;
}

}  // namespace

extern "C" {

JNIEXPORT jlong JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeCreate(JNIEnv* env, jobject,
                                                 jstring configJson) {
  const RobotConfig cfg =
      RobotConfig::fromJson(FlatJson::parse(jstringToString(env, configJson)));
  // Not aggregate-init: the first member is a mutex, {cfg} would try to
  // construct it from the config.
  NavHolder* n = new NavHolder();
  n->cfg = cfg;
  return reinterpret_cast<jlong>(n);
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeDestroy(JNIEnv*, jobject,
                                                  jlong handle) {
  delete fromNav(handle);
}

JNIEXPORT jboolean JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeSetMap(JNIEnv* env, jobject,
                                                 jlong handle,
                                                 jbyteArray snapshot) {
  NavHolder* n = fromNav(handle);
  if (!n || !snapshot) return JNI_FALSE;
  const jsize len = env->GetArrayLength(snapshot);
  std::vector<uint8_t> buf(static_cast<size_t>(len));
  env->GetByteArrayRegion(snapshot, 0, len,
                          reinterpret_cast<jbyte*>(buf.data()));
  std::lock_guard<std::mutex> lock(n->mutex);
  return n->setMap(buf.data(), buf.size()) ? JNI_TRUE : JNI_FALSE;
}

// rects: [x, y, w, h] quads in world metres (NoGoZone / virtual walls).
JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeSetKeepouts(JNIEnv* env, jobject,
                                                      jlong handle,
                                                      jfloatArray rects) {
  NavHolder* n = fromNav(handle);
  if (!n || !rects) return;
  const jsize len = env->GetArrayLength(rects);
  std::vector<float> r(static_cast<size_t>(len));
  env->GetFloatArrayRegion(rects, 0, len, r.data());
  std::lock_guard<std::mutex> lock(n->mutex);
  n->cm.clearKeepouts();
  for (jsize i = 0; i + 3 < len; i += 4) {
    const double x = r[i], y = r[i + 1], w = r[i + 2], h = r[i + 3];
    n->cm.addKeepout({{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}});
  }
}

// xy: flat world points from the latest scan (Kotlin applies the lidar
// transform); they go into the decaying live-obstacle layer.
JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeSetObstacles(JNIEnv* env, jobject,
                                                       jlong handle,
                                                       jfloatArray xy,
                                                       jint count) {
  NavHolder* n = fromNav(handle);
  if (!n || !xy) return;
  std::vector<float> pts(static_cast<size_t>(count) * 2);
  env->GetFloatArrayRegion(xy, 0, count * 2, pts.data());
  std::lock_guard<std::mutex> lock(n->mutex);
  std::vector<vacuum::Vec2> world;
  world.reserve(static_cast<size_t>(count));
  for (jint i = 0; i < count; ++i)
    world.push_back({pts[i * 2], pts[i * 2 + 1]});
  n->cm.setObstacles(world, n->obstClockS);
}

// 0 = accepted, 1 = rejected (goal in lethal - the UI shows a clear
// message, prompt §7).
JNIEXPORT jint JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeSetGoal(JNIEnv*, jobject,
                                                  jlong handle, jfloat x,
                                                  jfloat y) {
  NavHolder* n = fromNav(handle);
  if (!n) return 1;
  std::lock_guard<std::mutex> lock(n->mutex);
  if (!n->hasMap) return 1;
  return n->gc.setGoal(n->cm, x, y) ? 0 : 1;
}

JNIEXPORT void JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeCancel(JNIEnv*, jobject,
                                                 jlong handle) {
  NavHolder* n = fromNav(handle);
  if (!n) return;
  std::lock_guard<std::mutex> lock(n->mutex);
  n->gc.cancel();
}

// One control tick: advances the virtual clock, expires stale obstacles,
// steps the state machine. Returns [v, w, stateCode].
JNIEXPORT jfloatArray JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeStep(JNIEnv* env, jobject,
                                               jlong handle, jfloat px,
                                               jfloat py, jfloat pth) {
  NavHolder* n = fromNav(handle);
  jfloat out[3] = {0.f, 0.f, 0.f};
  if (n) {
    std::lock_guard<std::mutex> lock(n->mutex);
    n->nowMs += 50;
    n->obstClockS += 0.05;
    n->cm.expire(n->obstClockS);   // live obstacles fade after2 s
    const auto cmd = n->gc.step(n->cm, vacuum::Pose2D(px, py, pth), n->nowMs);
    out[0] = static_cast<jfloat>(cmd.v);
    out[1] = static_cast<jfloat>(cmd.w);
    out[2] = static_cast<jfloat>(stateCode(cmd.state));
  }
  jfloatArray arr = env->NewFloatArray(3);
  env->SetFloatArrayRegion(arr, 0, 3, out);
  return arr;
}

// Flattened path [x0, y0, x1, y1, ...] for the map overlay.
JNIEXPORT jfloatArray JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativePath(JNIEnv* env, jobject,
                                               jlong handle) {
  NavHolder* n = fromNav(handle);
  if (!n) return env->NewFloatArray(0);
  std::lock_guard<std::mutex> lock(n->mutex);
  const auto& p = n->gc.path();
  std::vector<jfloat> flat(p.size() * 2);
  for (size_t i = 0; i < p.size(); ++i) {
    flat[i * 2] = static_cast<jfloat>(p[i].x);
    flat[i * 2 + 1] = static_cast<jfloat>(p[i].y);
  }
  jfloatArray arr = env->NewFloatArray(static_cast<jsize>(flat.size()));
  env->SetFloatArrayRegion(arr, 0, static_cast<jsize>(flat.size()),
                           flat.data());
  return arr;
}

// Phase 6: best frontier goal from the current pose, or null when every
// reachable frontier is smaller than 0.5 m (exploration finished).
JNIEXPORT jfloatArray JNICALL
Java_com_vacuum_nav_robot_NavEngine_nativeExploreGoal(JNIEnv* env, jobject,
                                                      jlong handle,
                                                      jfloat px, jfloat py) {
  NavHolder* n = fromNav(handle);
  if (!n) return nullptr;
  std::lock_guard<std::mutex> lock(n->mutex);
  if (!n->hasMap) return nullptr;
  const vacuum::FrontierExplorer ex;
  const auto goal = ex.bestGoal(n->grid, vacuum::Pose2D(px, py, 0.0));
  if (!goal.has_value()) return nullptr;
  jfloat out[2] = {static_cast<jfloat>(goal->x),
                   static_cast<jfloat>(goal->y)};
  jfloatArray arr = env->NewFloatArray(2);
  env->SetFloatArrayRegion(arr, 0, 2, out);
  return arr;
}

}  // extern "C"
