#include "opencv2/opencv.hpp"
#include <chrono>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <vector>

#include "cnpy.h"

#include <grpcpp/ext/proto_server_reflection_plugin.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/health_check_service_interface.h>

#include "camera_service.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;

class FrameProvider {
public:
  cv::Mat GetImage() {
    std::lock_guard<std::mutex> guard(last_image_mutex_);
    return last_image_.clone();
  }

  void Run() {
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
      std::cout << "Error opening video stream or file" << std::endl;
      return;
    }

    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 1200);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1600);

    while (true) {
      cv::Mat last_image;

      cap >> last_image;

      if (last_image.empty()) {
        continue;
      }

      std::lock_guard<std::mutex> guard(last_image_mutex_);
      last_image_ = last_image;
    }
  }

  void WaitForImage() {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      std::lock_guard<std::mutex> guard(last_image_mutex_);
      if (!last_image_.empty()) {
        return;
      }
    }
  }

private:
  std::mutex last_image_mutex_;
  cv::Mat last_image_;
};

cv::Mat ReadNpy(const std::string &file) {
  auto np_array = cnpy::npy_load(file);
  assert(np_array.shape.size() == 2);

  return cv::Mat(np_array.shape[0], np_array.shape[1], CV_64F,
                 np_array.data<double>())
      .clone();
}

constexpr float MIN_TRACKING_Y = 220;

constexpr int POSITION_BUFFER_SIZE = 30 * 2;
constexpr std::chrono::duration LOST_OBJECT_TRACK_TIMEOUT =
    std::chrono::seconds(5);

class DetectedObjectInternal {
public:
  DetectedObject GetProto() const {
    DetectedObject proto;
    proto.set_x(x);
    proto.set_y(y);
    proto.set_dx(dx);
    proto.set_dy(dy);
    proto.set_time(time);
    proto.set_length(length);
    return proto;
  }
  float x = 0, y = 0, dx = 0, dy = 0;
  double time;
  float length = 0;

  float x_buffer[POSITION_BUFFER_SIZE];
  float y_buffer[POSITION_BUFFER_SIZE];
  double time_buffer[POSITION_BUFFER_SIZE];
  int buffer_size = 0;

  std::chrono::time_point<std::chrono::system_clock> last_seen_time;
};

class ObjectsProvider {
public:
  ObjectsProvider(FrameProvider &frame_provider)
      : frame_provider_(frame_provider),
        camera_matrix_(ReadNpy("/home/user/delta/python/camera_matrix.npy")),
        distortion_(ReadNpy("/home/user/delta/python/distortion.npy")),
        camera_rotation_(
            ReadNpy("/home/user/delta/python/camera_rotation.npy")),
        camera_translation_(
            ReadNpy("/home/user/delta/python/camera_translation.npy")),
        camera_matrix_inv_(camera_matrix_.inv()),
        camera_rotation_inv_(camera_rotation_.inv()),
        last_log_(std::chrono::system_clock::now()) {}

  cv::Point2d FindXY(double u, double v) {
    double z = 0;

    cv::Mat uv = (cv::Mat_<double>(3, 1) << u, v, 1.0);

    cv::Mat lsm = (camera_rotation_inv_ * camera_matrix_inv_) * uv;
    cv::Mat rsm = camera_rotation_inv_ * camera_translation_;

    double s = (z + rsm.at<double>(2, 0)) / lsm.at<double>(2, 0);
    cv::Mat p = camera_rotation_inv_ *
                (s * (camera_matrix_inv_ * uv) - camera_translation_);
    return cv::Point2d(p.at<double>(0, 0), p.at<double>(1, 0));
  }

  std::vector<DetectedObject> GetDetectedObjects() {
    std::lock_guard<std::mutex> guard(detected_objects_mutex_);
    std::vector<DetectedObject> objects(detected_objects_.size());
    std::transform(
        detected_objects_.begin(), detected_objects_.end(), objects.begin(),
        [](const DetectedObjectInternal &detected_object) -> DetectedObject {
          return detected_object.GetProto();
        });
    return objects;
  }

  float Distance(const DetectedObjectInternal &o1,
                 const DetectedObjectInternal &o2) {
    return std::sqrt((o1.x - o2.x) * (o1.x - o2.x) +
                     (o1.y - o2.y) * (o1.y - o2.y));
  }

  void Run() {
    while (true) {
      // std::this_thread::sleep_for(std::chrono::milliseconds(100));
      auto distorted_image = frame_provider_.GetImage();
      auto now_time_point = std::chrono::system_clock::now();
      double time =
          ((double)std::chrono::duration_cast<std::chrono::milliseconds>(
               now_time_point.time_since_epoch())
               .count()) /
          1000.0;
      cv::Mat image;
      cv::undistort(distorted_image, image, camera_matrix_, distortion_);

      cv::Rect crop_rect(100, 100, 1400, 900);
      cv::Mat image_cropped = image(crop_rect);

      cv::Mat gray_image;
      cv::cvtColor(image_cropped, gray_image, cv::COLOR_BGR2GRAY);
      cv::Mat binary_image;
      cv::threshold(gray_image, binary_image, 160, 255, cv::THRESH_BINARY);
      cv::Mat max_image;
      cv::dilate(binary_image, max_image, cv::Mat());
      std::vector<std::vector<cv::Point>> contours;
      cv::findContours(max_image, contours, cv::RETR_LIST,
                       cv::CHAIN_APPROX_SIMPLE);

      // cv::imwrite("a.png", distorted_image);
      // cv::imwrite("b.png", gray_image);
      // cv::imwrite("c.png", binary_image);
      // cv::imwrite("d.png", max_image);

      std::vector<DetectedObjectInternal> objects;
      std::vector<DetectedObjectInternal> old_objects(detected_objects_);

      for (const auto &contour : contours) {
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        if (radius > 50) {
          auto center_pt =
              FindXY(center.x + crop_rect.x, center.y + crop_rect.y);
          auto edge_pt =
              FindXY(center.x + crop_rect.x + radius, center.y + crop_rect.y);
          if (center_pt.y > MIN_TRACKING_Y) {
            DetectedObjectInternal object;
            object.x = center_pt.x;
            object.y = center_pt.y;
            object.length = 2 * std::sqrt(std::pow(center_pt.x - edge_pt.x, 2) +
                                          std::pow(center_pt.y - edge_pt.y, 2));
            object.time = time;
            object.last_seen_time = now_time_point;

            for (int i = 0; i < old_objects.size(); i++) {
              if (Distance(old_objects[i], object) < 10) {
                object.x_buffer[0] = old_objects[i].x;
                object.y_buffer[0] = old_objects[i].y;
                object.time_buffer[0] = old_objects[i].time;

                for (int j = 0; j < std::min(old_objects[i].buffer_size,
                                             POSITION_BUFFER_SIZE - 1);
                     j++) {
                  object.x_buffer[j + 1] = old_objects[i].x_buffer[j];
                  object.y_buffer[j + 1] = old_objects[i].y_buffer[j];
                  object.time_buffer[j + 1] = old_objects[i].time_buffer[j];
                }
                object.buffer_size = std::min(1 + old_objects[i].buffer_size,
                                              POSITION_BUFFER_SIZE);

                object.dx =
                    (object.x - object.x_buffer[object.buffer_size - 1]) /
                    (object.time - object.time_buffer[object.buffer_size - 1]);
                object.dy =
                    (object.y - object.y_buffer[object.buffer_size - 1]) /
                    (object.time - object.time_buffer[object.buffer_size - 1]);
                old_objects.erase(old_objects.begin() + i);
                break;
              }
            }

            objects.push_back(object);
          }
        }
      }

      for (auto old_object : old_objects) {
        if (now_time_point - old_object.last_seen_time <
            LOST_OBJECT_TRACK_TIMEOUT) {
          old_object.x =
              old_object.x + old_object.dx * (time - old_object.time);
          old_object.y =
              old_object.y + old_object.dy * (time - old_object.time);
          old_object.time = time;
          objects.push_back(old_object);
        }
      }

      auto now = std::chrono::system_clock::now();
      if (now - last_log_ > std::chrono::milliseconds(300)) {
        last_log_ = now;
        std::cout << "-------------------------------\n";
        for (const auto &object : objects) {
          std::cout << object.GetProto().DebugString() << std::endl;
        }
      }

      std::lock_guard<std::mutex> guard(detected_objects_mutex_);
      detected_objects_ = objects;
      // std::cout << std::fixed << detected_objects_[0].time() <<
      // std::endl;
    }
  }

private:
  FrameProvider &frame_provider_;
  cv::Mat camera_matrix_;
  cv::Mat distortion_;
  cv::Mat camera_rotation_;
  cv::Mat camera_translation_;
  cv::Mat camera_matrix_inv_;
  cv::Mat camera_rotation_inv_;

  std::mutex detected_objects_mutex_;
  std::vector<DetectedObjectInternal> detected_objects_;
  std::chrono::time_point<std::chrono::system_clock> last_log_;
};

class CameraDetectionServiceImpl final
    : public CameraDetectionService::Service {
public:
  CameraDetectionServiceImpl(ObjectsProvider &objects_provider)
      : objects_provider_(objects_provider) {}

  Status Detect(ServerContext *context, const CameraDetectionRequest *request,
                CameraDetectionResponse *resposne) override {
    auto objects = objects_provider_.GetDetectedObjects();
    for (auto object : objects) {
      *resposne->add_detected_objects() = object;
    }
    return Status::OK;
  }

private:
  ObjectsProvider &objects_provider_;
};

void RunServer(ObjectsProvider &objects_provider) {
  std::string server_address("0.0.0.0:5555");
  CameraDetectionServiceImpl service(objects_provider);

  grpc::EnableDefaultHealthCheckService(true);
  grpc::reflection::InitProtoReflectionServerBuilderPlugin();
  ServerBuilder builder;
  // Listen on the given address without any authentication mechanism.
  builder.AddListeningPort(server_address, grpc::InsecureServerCredentials());
  // Register "service" as the instance through which we'll communicate with
  // clients. In this case it corresponds to an *synchronous* service.
  builder.RegisterService(&service);
  // Finally assemble the server.
  std::unique_ptr<Server> server(builder.BuildAndStart());
  std::cout << "Server listening on " << server_address << std::endl;

  // Wait for the server to shutdown. Note that some other thread must be
  // responsible for shutting down the server for this call to ever return.
  server->Wait();
}

int main() {
  FrameProvider frame_provider;
  std::thread camera_thread(&FrameProvider::Run, &frame_provider);
  frame_provider.WaitForImage();

  ObjectsProvider objects_provider(frame_provider);
  std::thread detection_thread(&ObjectsProvider::Run, &objects_provider);

  RunServer(objects_provider);

  return 0;
}
