package frc.robot;

import static java.util.Map.entry;

import java.util.Map;

public final class Ports {
  public static final Map<Integer, String> idToName =
      Map.ofEntries(
          entry(Shooter.TOP_LEADER, "top shooter"),
          entry(Shooter.BOTTOM_FOLLOWER, "bottom shooter"));

  public static final class Shooter {
    // TODO make sure leader/follower is accurate
    public static final int TOP_LEADER = 31;
    public static final int BOTTOM_FOLLOWER = 32;
  }
}
