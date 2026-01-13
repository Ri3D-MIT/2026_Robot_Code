package frc.robot;

import static java.util.Map.entry;
import java.util.Map;

public final class Ports {
  public static final Map<Integer, String> idToName = 
    Map.ofEntries(
      entry(Elevator.ELEVATOR_LEADER, "elevator left"),
      entry(Elevator.ELEVATOR_FOLLOWER, "elevator right"),
      entry(Elevator.LOWER_SWITCH_CHANNEL, "lower limit switch"),
      entry(Elevator.UPPER_SWITCH_CHANNEL, "upper limit switch")
    );

  public static final class Elevator {
    public static final int ELEVATOR_LEADER = 37;
    public static final int ELEVATOR_FOLLOWER = 38;
    public static final int LOWER_SWITCH_CHANNEL = 1;
    public static final int UPPER_SWITCH_CHANNEL = 2;
  }
}
