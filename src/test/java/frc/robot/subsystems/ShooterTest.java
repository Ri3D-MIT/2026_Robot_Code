package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.lib.UnitTestingUtil.fastForward;
import static frc.lib.UnitTestingUtil.reset;
import static frc.lib.UnitTestingUtil.run;
import static frc.lib.UnitTestingUtil.setupTests;
import static org.junit.jupiter.api.Assertions.assertEquals;

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

  @Test
  void idle() {
    assertEquals(0, shooter.velocity(), 10);
    fastForward(Seconds.of(3));
    assertEquals(Shooter.IDLE_SPEED.in(RadiansPerSecond), shooter.velocity(), 10);
  }

  @Test
  void maxStop() {
    run(shooter.runShooter(Shooter.MAX_SPEED.in(RadiansPerSecond)));
    fastForward(Seconds.of(5));
    assertEquals(Shooter.MAX_SPEED.in(RadiansPerSecond), shooter.velocity(), 20);
    run(shooter.stopMotors());
    fastForward(Seconds.of(3));
    assertEquals(0, shooter.velocity(), 10);
  }
}
