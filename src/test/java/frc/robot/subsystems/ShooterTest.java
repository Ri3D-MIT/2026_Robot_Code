package frc.robot.subsystems;

import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

public class ShooterTest {
  private Shooter shooter;

  @BeforeEach
  void setup() {
    shooter = new Shooter();
    setupTests();
  }

  @AfterEach
  void destroy() throws Exception {
    reset(shooter);
  }

  @Test
  void init() {}

  @ParameterizedTest
  @ValueSource(doubles = {-200, -100, -50, 0, 50, 100, 200, 300})
  void goToTest(double velocity) {
    frc.lib.Test.runUnitTest(shooter.goToTest(velocity));
  }
}
