package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

public final class Ports {
  public static final Map<Integer, String> idToName =
      Map.ofEntries(
          entry(Shooter.TOP_LEADER, "top shooter"),
          entry(Feeder.FEEDER, "feeder"),
          entry(Elevator.ELEVATOR_LEADER, "elevator left"),
          entry(Elevator.ELEVATOR_FOLLOWER, "elevator right"),
          entry(Intake.INTAKE_MOTOR, "intake motor"),
          entry(Intake.PIVOT_MOTOR, "pivot motor"));

  public static final class Shooter {
    // public static final int BOTTOM_FOLLOWER = 31;
    public static final int TOP_LEADER = 32;
  }

  public static final class Feeder {
    public static final int FEEDER = 31;
  }

  public static final class Elevator {
    public static final int ELEVATOR_LEADER = 37;
    public static final int ELEVATOR_FOLLOWER = 38;
  }

  public static final class Intake {
    public static final int INTAKE_MOTOR = 39;
    public static final int PIVOT_MOTOR = 36;
  }
}