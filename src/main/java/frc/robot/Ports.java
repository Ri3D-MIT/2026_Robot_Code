package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

public final class Ports {
  public static final Map<Integer, String> idToName =
      Map.ofEntries(
          entry(Shooter.TOP_LEADER, "top shooter"),
          entry(Shooter.BOTTOM_FOLLOWER, "bottom shooter"),
          entry(Feeder.FEEDER, "feeder"),
          entry(Elevator.ELEVATOR_LEADER, "elevator left"),
          entry(Elevator.ELEVATOR_FOLLOWER, "elevator right"));

  public static final class Shooter {
    public static final int TOP_LEADER = 31;
    public static final int BOTTOM_FOLLOWER = 32;
  }

  public static final class Feeder {
    public static final int FEEDER = 39;
  }

  public static final class Elevator {
    public static final int ELEVATOR_LEADER = 37;
    public static final int ELEVATOR_FOLLOWER = 38;
  }
}
