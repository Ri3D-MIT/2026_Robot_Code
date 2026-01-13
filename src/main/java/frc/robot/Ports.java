package frc.robot;

import java.util.Map;
import static java.util.Map.entry;


public final class Ports {
  public static final Map<Integer, String> idToName = 
    Map.ofEntries(
      entry(Intake.INTAKE_MOTOR, "intake motor"),
      entry(Intake.PIVOT_MOTOR, "pivot motor"));

  public static final class Intake {
      public static final int INTAKE_MOTOR = 35;
      public static final int PIVOT_MOTOR = 36;
  } 
}
