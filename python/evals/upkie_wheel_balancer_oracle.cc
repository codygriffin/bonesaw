// Executes the pinned upstream Upkie WheelBalancer class without modifying it.
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "upkie/cpp/controllers/WheelBalancer.h"

using palimpsest::Dictionary;
using upkie::cpp::controllers::WheelBalancer;

struct Sample {
  double ground_position;
  double pitch;
};

struct Record {
  double ground_velocity;
  double left_wheel_velocity;
  double right_wheel_velocity;
  std::uint64_t elapsed_ns;
};

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: upkie-wheel-balancer-oracle INPUT.tsv OUTPUT.tsv\n";
    return 2;
  }
  std::ifstream input(argv[1]);
  if (!input) {
    std::cerr << "cannot open input corpus\n";
    return 2;
  }
  std::string header;
  std::getline(input, header);
  std::vector<Sample> samples;
  Sample sample{};
  while (input >> sample.ground_position >> sample.pitch) {
    samples.push_back(sample);
  }

  WheelBalancer::Parameters parameters;
  parameters.dt = 0.005;
  WheelBalancer controller(parameters);
  Dictionary config;
  controller.reset(config);

  Dictionary observation;
  Dictionary action;
  observation("base_orientation")("pitch") = 0.0;
  observation("wheel_odometry")("position") = 0.0;
  observation("floor_contact")("contact") = true;
  action("servo")("left_wheel")("velocity") = 0.0;
  action("servo")("right_wheel")("velocity") = 0.0;
  std::vector<Record> records;
  records.reserve(samples.size());

  for (const auto& current : samples) {
    observation("base_orientation")("pitch") = current.pitch;
    observation("wheel_odometry")("position") = current.ground_position;
    action("servo")("left_wheel")("velocity") = 0.0;
    action("servo")("right_wheel")("velocity") = 0.0;
    const auto started = std::chrono::steady_clock::now();
    controller.read(observation, action);
    controller.write(action);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    const double left = action("servo")("left_wheel")("velocity");
    const double right = action("servo")("right_wheel")("velocity");
    records.push_back({
        left * parameters.wheel_radius,
        left,
        right,
        static_cast<std::uint64_t>(
            std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed)
                .count()),
    });
  }

  std::ofstream output(argv[2]);
  output << "ground_velocity\tleft_wheel_velocity\tright_wheel_velocity"
            "\telapsed_ns\n";
  output << std::setprecision(17) << std::scientific;
  for (const auto& record : records) {
    output << record.ground_velocity << '\t' << record.left_wheel_velocity
           << '\t' << record.right_wheel_velocity << '\t'
           << record.elapsed_ns << '\n';
  }
}
