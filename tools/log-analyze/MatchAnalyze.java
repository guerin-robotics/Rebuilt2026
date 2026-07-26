import edu.wpi.first.util.datalog.DataLogReader;
import edu.wpi.first.util.datalog.DataLogRecord;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * Drivetrain metrics from AdvantageKit match logs. See README.md for how to run it and for the
 * baseline numbers measured at the July 2026 Indiana event.
 *
 * <p>Every velocity metric is computed in the field frame. The logged chassis speeds are robot
 * relative, so differentiating them directly makes the frame's own rotation look like acceleration
 * — at 5 rad/s and 3 m/s that fabricates 15 m/s^2 of nonexistent acceleration.
 */
public final class MatchAnalyze {
  /** One logged sample: timestamp plus up to three payload fields. */
  private record Sample(double t, double a, double b, double c) {}

  private static final double ACCEL_CEILING = 5.5; // DriveCommands slew limit, m/s^2
  private static final double BRAKE_BUDGET = 7.0; // DriveCommands limiter, m/s^2
  private static final double SLIP_CURRENT = 80.0; // configured stator limit, A

  private static final Set<String> WANTED =
      new HashSet<>(
          List.of(
              "/RealOutputs/SwerveChassisSpeeds/Setpoints",
              "/RealOutputs/SwerveChassisSpeeds/Measured",
              "/RealOutputs/Odometry/Robot",
              "/RealOutputs/Odometry/TrajectorySetpoint",
              "/RealOutputs/BatteryLogger/Current",
              "/RealOutputs/BatteryLogger/Current/Drive/",
              "/RealOutputs/BatteryLogger/Current/Flywheel",
              "/SystemStats/BatteryVoltage",
              "/SystemStats/BrownedOut",
              "/Drive/Module0/DriveCurrentAmps",
              "/Drive/Module1/DriveCurrentAmps",
              "/Drive/Module2/DriveCurrentAmps",
              "/Drive/Module3/DriveCurrentAmps"));

  private final Map<String, List<Sample>> data = new HashMap<>();
  private final List<double[]> dsAuto = new ArrayList<>();
  private final List<double[]> dsEnabled = new ArrayList<>();

  public static void main(String[] args) throws Exception {
    if (args.length == 0) {
      System.out.println("usage: MatchAnalyze <log.wpilog> [more.wpilog ...]");
      return;
    }
    for (String path : args) {
      MatchAnalyze m = new MatchAnalyze();
      m.load(path);
      System.out.println();
      System.out.println(
          "########## " + path.substring(path.lastIndexOf((char) 92) + 1) + " ##########");
      m.reportCommandShaping();
      m.reportCurrent();
      m.reportBattery();
      m.reportPathError();
    }
  }

  /** Reads the log, keeping only the entries this tool reports on. */
  private void load(String path) throws Exception {
    DataLogReader reader = new DataLogReader(path);
    Map<Integer, String> names = new HashMap<>();
    Map<Integer, String> types = new HashMap<>();
    var it = reader.iterator();
    while (it.hasNext()) {
      DataLogRecord r;
      try {
        r = it.next();
      } catch (RuntimeException truncated) {
        // Match logs are routinely cut off mid-record when the robot loses power
        break;
      }
      if (r.isStart()) {
        names.put(r.getStartData().entry, r.getStartData().name);
        types.put(r.getStartData().entry, r.getStartData().type);
        continue;
      }
      if (r.isFinish() || r.isSetMetadata()) {
        continue;
      }
      String name = names.get(r.getEntry());
      if (name == null) {
        continue;
      }
      double t = r.getTimestamp() / 1e6;
      if (name.equals("/DriverStation/Autonomous")) {
        dsAuto.add(new double[] {t, r.getBoolean() ? 1 : 0});
        continue;
      }
      if (name.equals("/DriverStation/Enabled")) {
        dsEnabled.add(new double[] {t, r.getBoolean() ? 1 : 0});
        continue;
      }
      if (!WANTED.contains(name)) {
        continue;
      }
      String type = types.get(r.getEntry());
      if (type.startsWith("struct:")) {
        ByteBuffer bb = ByteBuffer.wrap(r.getRaw()).order(ByteOrder.LITTLE_ENDIAN);
        if (bb.remaining() >= 24) {
          add(name, new Sample(t, bb.getDouble(), bb.getDouble(), bb.getDouble()));
        }
      } else if (type.equals("double")) {
        add(name, new Sample(t, r.getDouble(), 0, 0));
      } else if (type.equals("boolean")) {
        add(name, new Sample(t, r.getBoolean() ? 1 : 0, 0, 0));
      }
    }
  }

  private void add(String name, Sample s) {
    data.computeIfAbsent(name, k -> new ArrayList<>()).add(s);
  }

  private List<Sample> get(String name) {
    return data.getOrDefault(name, List.of());
  }

  /** Latest value of a boolean event stream at time t. */
  private static boolean stateAt(List<double[]> events, double t) {
    boolean v = false;
    for (double[] e : events) {
      if (e[0] > t) {
        break;
      }
      v = e[1] > 0.5;
    }
    return v;
  }

  /** Robot heading at time t, from the logged pose stream. */
  private double headingAt(double t) {
    List<Sample> pose = get("/RealOutputs/Odometry/Robot");
    if (pose.isEmpty()) {
      return 0;
    }
    int lo = 0;
    int hi = pose.size() - 1;
    while (lo < hi) {
      int mid = (lo + hi) / 2;
      if (pose.get(mid).t() < t) {
        lo = mid + 1;
      } else {
        hi = mid;
      }
    }
    return pose.get(lo).c();
  }

  private static double percentile(List<Double> sorted, double p) {
    if (sorted.isEmpty()) {
      return 0;
    }
    return sorted.get(Math.min(sorted.size() - 1, (int) (p * sorted.size())));
  }

  /**
   * How abruptly the drive was commanded. Large values mean the command asked for a velocity step
   * the hardware could only answer with a current spike.
   */
  private void reportCommandShaping() {
    for (String mode : new String[] {"AUTO", "TELEOP"}) {
      List<Double> accel = new ArrayList<>();
      List<Double> brake = new ArrayList<>();
      List<Double> steps = new ArrayList<>();
      double prevX = 0;
      double prevY = 0;
      double prevT = 0;
      boolean have = false;
      for (Sample q : get("/RealOutputs/SwerveChassisSpeeds/Setpoints")) {
        double th = headingAt(q.t());
        double fx = q.a() * Math.cos(th) - q.b() * Math.sin(th);
        double fy = q.a() * Math.sin(th) + q.b() * Math.cos(th);
        double dt = q.t() - prevT;
        boolean inMode = mode.equals("AUTO") == stateAt(dsAuto, q.t());
        if (have && dt > 0.005 && dt <= 0.1 && stateAt(dsEnabled, q.t()) && inMode) {
          double step = Math.hypot(fx - prevX, fy - prevY);
          steps.add(step);
          if (Math.hypot(fx, fy) < Math.hypot(prevX, prevY)) {
            brake.add(step / dt);
          } else {
            accel.add(step / dt);
          }
        }
        prevX = fx;
        prevY = fy;
        prevT = q.t();
        have = true;
      }
      if (accel.isEmpty() && brake.isEmpty()) {
        continue;
      }
      Collections.sort(accel);
      Collections.sort(brake);
      Collections.sort(steps);
      long overCeiling = accel.stream().filter(x -> x > ACCEL_CEILING).count();
      long overBrake = brake.stream().filter(x -> x > BRAKE_BUDGET).count();
      System.out.printf(
          "  %-6s commanded accel p50=%.1f p95=%.1f max=%.1f m/s^2 | %.0f%% above the %.1f ceiling%n",
          mode,
          percentile(accel, .50),
          percentile(accel, .95),
          accel.isEmpty() ? 0 : accel.get(accel.size() - 1),
          100.0 * overCeiling / Math.max(1, accel.size()),
          ACCEL_CEILING);
      System.out.printf(
          "  %-6s commanded brake p50=%.1f p95=%.1f max=%.1f m/s^2 | %.0f%% above the %.1f budget%n",
          mode,
          percentile(brake, .50),
          percentile(brake, .95),
          brake.isEmpty() ? 0 : brake.get(brake.size() - 1),
          100.0 * overBrake / Math.max(1, brake.size()),
          BRAKE_BUDGET);
      System.out.printf(
          "  %-6s command step p50=%.3f p95=%.3f m/s per update%n",
          mode, percentile(steps, .50), percentile(steps, .95));
    }
  }

  /** Drive stator current against the configured slip limit. */
  private void reportCurrent() {
    int atLimit = 0;
    int total = 0;
    double peak = 0;
    for (int m = 0; m < 4; m++) {
      for (Sample s : get("/Drive/Module" + m + "/DriveCurrentAmps")) {
        double i = Math.abs(s.a());
        total++;
        peak = Math.max(peak, i);
        if (i >= SLIP_CURRENT) {
          atLimit++;
        }
      }
    }
    if (total == 0) {
      return;
    }
    System.out.printf(
        "  drive stator current: peak %.0f A | %.2f%% of samples at or above the %.0f A limit%n",
        peak, 100.0 * atLimit / total, SLIP_CURRENT);
  }

  /**
   * Brownouts split by match mode, and what was drawing current when they happened.
   *
   * <p>Caveat: BatteryLogger sums stator currents, which only approach battery draw at high duty
   * cycle. A mechanism running fast (flywheel) is represented more faithfully than one making peak
   * torque at low speed (drive), so treat the shares as directional, not exact.
   */
  private void reportBattery() {
    double minAuto = 99;
    double minTele = 99;
    for (Sample v : get("/SystemStats/BatteryVoltage")) {
      if (!stateAt(dsEnabled, v.t())) {
        continue;
      }
      if (stateAt(dsAuto, v.t())) {
        minAuto = Math.min(minAuto, v.a());
      } else {
        minTele = Math.min(minTele, v.a());
      }
    }

    List<Sample> flags = get("/SystemStats/BrownedOut");
    int inAuto = 0;
    int inTeleop = 0;
    double sumTotal = 0;
    double sumDrive = 0;
    double sumFly = 0;
    int counted = 0;
    for (int i = 1; i < flags.size(); i++) {
      if (!(flags.get(i).a() > 0.5 && flags.get(i - 1).a() < 0.5)) {
        continue;
      }
      double bt = flags.get(i).t();
      if (!stateAt(dsEnabled, bt)) {
        continue;
      }
      if (stateAt(dsAuto, bt)) {
        inAuto++;
      } else {
        inTeleop++;
      }
      sumTotal += peakBefore("/RealOutputs/BatteryLogger/Current", bt);
      sumDrive += peakBefore("/RealOutputs/BatteryLogger/Current/Drive/", bt);
      sumFly += peakBefore("/RealOutputs/BatteryLogger/Current/Flywheel", bt);
      counted++;
    }
    System.out.printf(
        "  battery: min %.2f V in auto, %.2f V in teleop | brownouts: %d auto, %d teleop%n",
        minAuto > 90 ? 0 : minAuto, minTele > 90 ? 0 : minTele, inAuto, inTeleop);
    if (counted > 0) {
      System.out.printf(
          "  at brownout (peak in prior 0.3 s): %.0f A accounted | drive %.0f A (%.0f%%) | flywheel %.0f A (%.0f%%)%n",
          sumTotal / counted,
          sumDrive / counted,
          100 * sumDrive / Math.max(1, sumTotal),
          sumFly / counted,
          100 * sumFly / Math.max(1, sumTotal));
    }
  }

  private double peakBefore(String entry, double t) {
    double peak = 0;
    for (Sample s : get(entry)) {
      if (s.t() > t - 0.3 && s.t() <= t) {
        peak = Math.max(peak, s.a());
      }
    }
    return peak;
  }

  /** Distance between where PathPlanner wanted the robot and where it actually was. */
  private void reportPathError() {
    List<Sample> target = get("/RealOutputs/Odometry/TrajectorySetpoint");
    List<Sample> actual = get("/RealOutputs/Odometry/Robot");
    if (target.isEmpty() || actual.isEmpty()) {
      return;
    }
    for (String mode : new String[] {"AUTO", "TELEOP"}) {
      List<Double> errs = new ArrayList<>();
      int j = 0;
      for (Sample s : target) {
        while (j < actual.size() - 1 && actual.get(j).t() < s.t()) {
          j++;
        }
        if (mode.equals("AUTO") != stateAt(dsAuto, s.t())) {
          continue;
        }
        errs.add(Math.hypot(actual.get(j).a() - s.a(), actual.get(j).b() - s.b()));
      }
      if (errs.isEmpty()) {
        continue;
      }
      List<Double> sorted = new ArrayList<>(errs);
      Collections.sort(sorted);
      System.out.printf(
          "  %-6s path-follow error: mean=%.3f p95=%.3f max=%.3f m (%d samples)%n",
          mode,
          errs.stream().mapToDouble(Double::doubleValue).average().orElse(0),
          percentile(sorted, .95),
          sorted.get(sorted.size() - 1),
          errs.size());
    }
  }
}
