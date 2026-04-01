export module mininav.core.types;

export namespace mininav {

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct Twist2D {
  double v{0.0};
  double w{0.0};
};

struct StateRecord {
  double t{0.0};
  Pose2D pose{};
  Twist2D twist{};
};

} // namespace mininav