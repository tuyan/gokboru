// main.cpp
#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <mutex>
#include <netinet/in.h>
#include <pthread.h>
#include <sched.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "mavlink/ardupilotmega/mavlink.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>

// --- Global State & Multi-threading ---
struct DroneState {
  double current_lat = 0.0;
  double current_lon = 0.0;
  float current_alt = 0.0;
  struct sockaddr_in gcs_addr = {};
  bool gcs_connected = false;
};

DroneState state;
std::mutex state_mutex;
std::atomic<bool> keep_running{true};

enum MissionState { STATE_TRANSIT, STATE_ORB_SEARCH, STATE_TRACKING };

// --- MAVLink Signing ---
mavlink_signing_t signing;
mavlink_signing_streams_t signing_streams;

uint64_t get_mavlink_time_10us() {
  using namespace std::chrono;
  auto now_micros =
      duration_cast<microseconds>(system_clock::now().time_since_epoch())
          .count();
  const uint64_t MICROS_TO_2015 = 1420070400000000ULL;
  if (now_micros < MICROS_TO_2015)
    return 0;
  return (now_micros - MICROS_TO_2015) / 10;
}

void enable_mavlink_signing() {
  const uint8_t secret_key[32] = {
      0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56,
      0x78, 0x90, 0xAB, 0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90, 0xAB,
      0xCD, 0xEF, 0x12, 0x34, 0x56, 0x78, 0x90, 0xAB, 0xCD, 0xEF};
  memset(&signing, 0, sizeof(signing));
  signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
  signing.link_id = 1;
  memcpy(signing.secret_key, secret_key, 32);
  signing.timestamp = get_mavlink_time_10us();
  mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
  status->signing = &signing;
  status->signing_streams = &signing_streams;
}

// --- PID Controller ---
class PIDController {
private:
  float kp, ki, kd, integral_error, previous_error, output_limit,
      integral_limit;

public:
  PIDController(float p, float i, float d, float max_v, float max_i)
      : kp(p), ki(i), kd(d), integral_error(0.0f), previous_error(0.0f),
        output_limit(max_v), integral_limit(max_i) {}

  float update(float setpoint, float measured_value, float dt) {
    if (dt <= 0.0f)
      return 0.0f;
    float error = setpoint - measured_value;
    float p_out = kp * error;
    integral_error += error * dt;
    integral_error =
        std::clamp(integral_error, -integral_limit, integral_limit);
    float d_out = kd * ((error - previous_error) / dt);
    previous_error = error;
    return std::clamp(p_out + (ki * integral_error) + d_out, -output_limit,
                      output_limit);
  }
  void reset() {
    integral_error = 0.0f;
    previous_error = 0.0f;
  }
};

// --- Math & Navigation Helpers ---
double get_distance_meters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double phi1 = lat1 * M_PI / 180.0, phi2 = lat2 * M_PI / 180.0;
  double d_phi = (lat2 - lat1) * M_PI / 180.0,
         d_lam = (lon2 - lon1) * M_PI / 180.0;
  double a = std::sin(d_phi / 2.0) * std::sin(d_phi / 2.0) +
             std::cos(phi1) * std::cos(phi2) * std::sin(d_lam / 2.0) *
                 std::sin(d_lam / 2.0);
  return R * (2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a)));
}

void goto_gps_coordinate(int sock, struct sockaddr_in &target_addr, double lat,
                         double lon, float alt) {
  signing.timestamp = get_mavlink_time_10us();
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_set_position_target_global_int_pack(
      255, 1, &msg, 0, 1, 1, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0x0FF8,
      lat * 1e7, lon * 1e7, alt, 0, 0, 0, 0, 0, 0, 0, 0);
  sendto(sock, buffer, mavlink_msg_to_send_buffer(buffer, &msg), 0,
         (struct sockaddr *)&target_addr, sizeof(target_addr));
}

void send_velocity_command(int sock, struct sockaddr_in &target_addr, float vx,
                           float vy, float vz) {
  signing.timestamp = get_mavlink_time_10us();
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_set_position_target_local_ned_pack(
      255, 1, &msg, 0, 1, 1, MAV_FRAME_BODY_NED, 0x0FC7, 0, 0, 0, vx, vy, vz, 0,
      0, 0, 0, 0);
  sendto(sock, buffer, mavlink_msg_to_send_buffer(buffer, &msg), 0,
         (struct sockaddr *)&target_addr, sizeof(target_addr));
}

void send_yaw_scan_command(int sock, struct sockaddr_in &target_addr,
                           float yaw_rate_rads) {
  signing.timestamp = get_mavlink_time_10us();
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_set_position_target_local_ned_pack(
      255, 1, &msg, 0, 1, 1, MAV_FRAME_BODY_NED, 0x07C7, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, yaw_rate_rads);
  sendto(sock, buffer, mavlink_msg_to_send_buffer(buffer, &msg), 0,
         (struct sockaddr *)&target_addr, sizeof(target_addr));
}

void drop_payload(int sock, struct sockaddr_in &target_addr, uint8_t channel,
                  uint16_t pwm) {
  signing.timestamp = get_mavlink_time_10us();
  mavlink_message_t msg;
  uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
  mavlink_msg_command_long_pack(255, 1, &msg, 1, 1, MAV_CMD_DO_SET_SERVO, 0,
                                channel, pwm, 0, 0, 0, 0, 0);
  sendto(sock, buffer, mavlink_msg_to_send_buffer(buffer, &msg), 0,
         (struct sockaddr *)&target_addr, sizeof(target_addr));
}

// --- Background Threads ---
void set_realtime_priority(pthread_t thread, int policy, int priority_offset) {
  struct sched_param params;
  params.sched_priority = sched_get_priority_max(policy) - priority_offset;
  pthread_setschedparam(thread, policy, &params);
}

void mavlink_telemetry_loop(int sock) {
  struct sockaddr_in in_addr = {};
  socklen_t fromlen = sizeof(in_addr);
  uint8_t buf[2048];
  auto last_hb = std::chrono::steady_clock::now();

  while (keep_running) {
    ssize_t recsize = recvfrom(sock, (void *)buf, sizeof(buf), 0,
                               (struct sockaddr *)&in_addr, &fromlen);
    if (recsize > 0) {
      mavlink_message_t msg;
      mavlink_status_t status;
      for (ssize_t i = 0; i < recsize; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
          std::lock_guard<std::mutex> lock(state_mutex);
          if (!state.gcs_connected && msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            state.gcs_addr = in_addr;
            state.gcs_connected = true;
          }
          if (msg.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) {
            mavlink_global_position_int_t pos;
            mavlink_msg_global_position_int_decode(&msg, &pos);
            state.current_lat = pos.lat / 1e7;
            state.current_lon = pos.lon / 1e7;
            state.current_alt = pos.alt / 1000.0f;
          }
        }
      }
    }
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(now - last_hb)
            .count() >= 1) {
      std::lock_guard<std::mutex> lock(state_mutex);
      if (state.gcs_connected) {
        signing.timestamp = get_mavlink_time_10us();
        mavlink_message_t msg;
        uint8_t ob[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_heartbeat_pack(255, 1, &msg, MAV_TYPE_GCS,
                                   MAV_AUTOPILOT_INVALID, MAV_MODE_MANUAL_ARMED,
                                   0, MAV_STATE_ACTIVE);
        sendto(sock, ob, mavlink_msg_to_send_buffer(ob, &msg), 0,
               (struct sockaddr *)&state.gcs_addr, sizeof(state.gcs_addr));
      }
      last_hb = now;
    }
  }
}

// --- Main Execution ---
int main(int argc, char **argv) {
  if (argc < 4) {
    std::cerr << "Usage: " << argv[0]
              << " <Target_Lat> <Target_Lon> <Reference_Image_Path>\n";
    return -1;
  }
  double tgt_lat = std::stod(argv[1]);
  double tgt_lon = std::stod(argv[2]);
  std::string image_path = argv[3];

  // Load reference image
  cv::Mat ref_image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
  if (ref_image.empty()) {
    std::cerr << "Error: Could not load reference image at " << image_path
              << std::endl;
    return -1;
  }

  set_realtime_priority(pthread_self(), SCHED_FIFO, 0);
  enable_mavlink_signing();

  int sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
  struct sockaddr_in locAddr = {};
  locAddr.sin_family = AF_INET;
  locAddr.sin_port = htons(14550);
  locAddr.sin_addr.s_addr = INADDR_ANY;
  bind(sock, (struct sockaddr *)&locAddr, sizeof(locAddr));

  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 100000;
  setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (const char *)&tv, sizeof tv);

  std::thread tele_thread(mavlink_telemetry_loop, sock);
  set_realtime_priority(tele_thread.native_handle(), SCHED_FIFO, 10);

  cv::VideoCapture cap(0);
  if (!cap.isOpened())
    return -1;
  float cx = cap.get(cv::CAP_PROP_FRAME_WIDTH) / 2.0f;
  float cy = cap.get(cv::CAP_PROP_FRAME_HEIGHT) / 2.0f;

  // Feature Matching & Tracker Initialization
  cv::Ptr<cv::ORB> orb = cv::ORB::create(500);
  cv::Ptr<cv::DescriptorMatcher> matcher =
      cv::DescriptorMatcher::create("BruteForce-Hamming");
  std::vector<cv::KeyPoint> ref_keypoints;
  cv::Mat ref_descriptors;
  orb->detectAndCompute(ref_image, cv::noArray(), ref_keypoints,
                        ref_descriptors);

  cv::Ptr<cv::Tracker> csrt_tracker;
  bool is_csrt_tracking = false;
  cv::Rect2d current_bbox;

  PIDController pid_yaw(0.001f, 0.0f, 0.0f, 1.0f, 1.0f);
  PIDController pid_alt(0.001f, 0.0f, 0.0f, 1.0f, 1.0f);
  PIDController pid_dist(0.002f, 0.0f, 0.0f, 1.5f, 1.0f);

  MissionState m_state = STATE_TRANSIT;
  bool mission_started = false;
  auto last_frame_time = std::chrono::steady_clock::now();

  while (keep_running) {
    cv::Mat frame;
    cap >> frame;
    if (frame.empty())
      continue;
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - last_frame_time).count();
    last_frame_time = now;

    bool target_detected = false;
    float bbox_cx = 0, bbox_cy = 0, bbox_w = 0;

    // 1. Try CSRT Tracking first
    if (is_csrt_tracking) {
      if (csrt_tracker->update(frame, current_bbox)) {
        target_detected = true;
        cv::rectangle(frame, current_bbox, cv::Scalar(0, 255, 255),
                      2); // Yellow box for CSRT
      } else {
        is_csrt_tracking = false; // Lost track, fallback to ORB
      }
    }

    // 2. Fallback to ORB Feature Matching
    if (!target_detected) {
      cv::Mat frame_gray;
      cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
      std::vector<cv::KeyPoint> frame_keypoints;
      cv::Mat frame_descriptors;
      orb->detectAndCompute(frame_gray, cv::noArray(), frame_keypoints,
                            frame_descriptors);

      if (!frame_descriptors.empty() && !ref_descriptors.empty()) {
        std::vector<cv::DMatch> matches;
        matcher->match(ref_descriptors, frame_descriptors, matches);

        double min_dist = 100.0;
        for (const auto &m : matches)
          if (m.distance < min_dist)
            min_dist = m.distance;

        std::vector<cv::DMatch> good_matches;
        for (const auto &m : matches) {
          if (m.distance <= std::max(2.0 * min_dist, 30.0))
            good_matches.push_back(m);
        }

        if (good_matches.size() >= 15) {
          std::vector<cv::Point2f> obj, scene;
          for (const auto &m : good_matches) {
            obj.push_back(ref_keypoints[m.queryIdx].pt);
            scene.push_back(frame_keypoints[m.trainIdx].pt);
          }

          cv::Mat H = cv::findHomography(obj, scene, cv::RANSAC);
          if (!H.empty()) {
            std::vector<cv::Point2f> obj_corners(4), scene_corners(4);
            obj_corners[0] = cv::Point2f(0, 0);
            obj_corners[1] = cv::Point2f((float)ref_image.cols, 0);
            obj_corners[2] =
                cv::Point2f((float)ref_image.cols, (float)ref_image.rows);
            obj_corners[3] = cv::Point2f(0, (float)ref_image.rows);
            cv::perspectiveTransform(obj_corners, scene_corners, H);

            cv::Rect orb_bbox = cv::boundingRect(scene_corners);
            orb_bbox &=
                cv::Rect(0, 0, frame.cols, frame.rows); // Clamp to frame

            if (orb_bbox.area() > 0) {
              target_detected = true;
              current_bbox = orb_bbox;
              cv::rectangle(frame, current_bbox, cv::Scalar(0, 255, 0),
                            2); // Green box for ORB

              // Re-init CSRT
              csrt_tracker = cv::TrackerCSRT::create();
              csrt_tracker->init(frame, current_bbox);
              is_csrt_tracking = true;
            }
          }
        }
      }
    }

    if (target_detected) {
      bbox_cx = current_bbox.x + current_bbox.width / 2.0f;
      bbox_cy = current_bbox.y + current_bbox.height / 2.0f;
      bbox_w = current_bbox.width;
      cv::circle(frame, cv::Point(bbox_cx, bbox_cy), 4, cv::Scalar(0, 0, 255),
                 -1);
    }

    struct sockaddr_in target_addr;
    bool is_connected;
    double c_lat, c_lon;
    {
      std::lock_guard<std::mutex> lock(state_mutex);
      target_addr = state.gcs_addr;
      is_connected = state.gcs_connected;
      c_lat = state.current_lat;
      c_lon = state.current_lon;
    }

    if (is_connected) {
      if (!mission_started) {
        goto_gps_coordinate(sock, target_addr, tgt_lat, tgt_lon, 15.0f);
        mission_started = true;
      }

      if (m_state == STATE_TRANSIT) {
        if (get_distance_meters(c_lat, c_lon, tgt_lat, tgt_lon) < 3.0)
          m_state = STATE_ORB_SEARCH;
      } else if (m_state == STATE_ORB_SEARCH) {
        if (target_detected)
          m_state = STATE_TRACKING;
        else
          send_yaw_scan_command(sock, target_addr, 0.25f);
      } else if (m_state == STATE_TRACKING) {
        if (target_detected) {
          float vy = pid_yaw.update(cx, bbox_cx, dt),
                vz = pid_alt.update(cy, bbox_cy, dt);
          float vx = pid_dist.update(300.0f, bbox_w, dt);
          send_velocity_command(sock, target_addr, vx, vy, -vz);

          if (bbox_w > 280.0f && std::abs(cx - bbox_cx) < 30 &&
              std::abs(cy - bbox_cy) < 30) {
            drop_payload(sock, target_addr, 9, 1900);
            m_state = STATE_TRANSIT; // Reset or RTL
          }
        } else {
          send_velocity_command(sock, target_addr, 0, 0, 0);
        }
      }
    }

    cv::imshow("Gökbörü Image Tracking", frame);
    if (cv::waitKey(1) == 'q') {
      keep_running = false;
      break;
    }
  }
  tele_thread.join();
  cap.release();
  cv::destroyAllWindows();
  return 0;
}
