package frc.robot;

import static frc.lib.UnitTestingUtil.reset;

import org.junit.jupiter.api.Test;

public class RobotTest {
  @Test
  void initialize() throws Exception {
    new Robot().close();
    reset();
  }
}
